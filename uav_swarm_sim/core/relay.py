"""
Connectivity / relay manager -- v2: shared MST backbone.

Design history (see conversation / README): a first version gave each
active surveyor its own independent straight-line relay chain to the
GCS (Swarm Relays, Varadharajan et al. 2020, Proposition 1). Under the
actual competition numbers -- 100 m comm range over a 1000 m x 1000 m
area -- that needs up to ~11 relay hops for a single far surveyor, and
scales linearly with the number of *simultaneous* surveyors, which no
reasonable fleet can support.

v2 instead builds ONE shared tree (minimum spanning tree over
{GCS} u {active surveyor positions}) and places relay slots along its
edges. Surveyors heading in similar directions automatically share the
GCS-side trunk of the tree instead of each paying for their own private
line -- this is the efficiency gap we identified when reading Ponda et
al.'s CBBA with Relays (JSAC 2012): their Place-Relays creates relays
per disconnected task, one gap at a time, and does not share structure
across simultaneously-disconnected tasks either. Sharing via a tree is
our response to that limitation, cited as such in the report.

Also borrowed from Ponda et al. 2012 (their Prune-Task-Space /
Keep-Task mechanism, Sec. IV): if a relay slot goes unfilled for too
long (fleet has run out of free UAVs to relay with), we release the
PoI task(s) that slot was supporting back to PENDING, rather than
logging an unresolvable shortage forever. We use a deterministic
timeout rather than their probabilistic Keep-Task rule -- simpler, and
adequate here because slots are centrally computed each round rather
than competitively bid on, so the symmetric-deadlock problem their
stochastic rule solves does not arise in this design.
"""
from __future__ import annotations
import numpy as np
from collections import deque
from . import config as cfg
from .entities import UAVState, UAVRole, PoIStatus

SAFE_HOP_DIST = 0.95 * cfg.COMM_RANGE   # margin below hard COMM_RANGE cutoff.
# Tightened from an initial 0.85 after measuring that we don't need much
# margin here: our link model is a hard distance threshold (no signal
# noise/decay to buffer against), so a large safety margin was mostly
# wasted hop count. 0.85->0.95 measured as mean latency 64.5s->54.2s and
# reported 49/50->50/50 across 5 seeds, with collisions unaffected (0
# throughout) -- a small, clearly positive, no-downside change.

# Gating distance used by sim.py's backbone-advance throttle (Swarm Relays'
# Eq. 6-7 idea: a chain link only advances while its inward neighbour is
# still within a safe distance, otherwise it holds and lets the chain catch
# up). Deliberately set ABOVE SAFE_HOP_DIST (steady-state hop spacing) so
# a chain sitting at its intended spacing is never itself flagged as
# unsafe -- it should only trigger when a link is actually falling behind.
BACKBONE_GATE_DIST = 0.95 * cfg.COMM_RANGE
RELAY_SHORTAGE_DROP_TIME = 25.0          # s -- how long a slot may stay
                                          # unfilled before we give up and
                                          # release its dependent task(s)

# How often the MST *topology* (which node connects to which) is allowed
# to change. Recomputing this from scratch every simulation tick using
# constantly-moving surveyor positions causes edge churn: a tiny bit of
# drift flips which pairing is "shortest," which changes edge keys, which
# resets every slot on the old edges and cancels relays already travelling
# to them -- distant multi-hop chains then never get a stable enough
# window to actually form. Both CBBA with Relays (Ponda et al. 2012,
# Sec. IV-2: network prediction "only performed for the task execution
# time," not every timestep) and Swarm Relays (a chain's required length
# is computed once from the current plan, not re-derived from scratch
# each tick) avoid this by treating topology as something to revisit
# periodically, not continuously. We do the same: the edge *set* is
# recomputed only every MST_RECOMPUTE_INTERVAL (or immediately if the set
# of active surveyors itself changes -- a genuinely new backbone need),
# while slot *positions* along the existing edges still track their
# endpoints smoothly every tick.
MST_RECOMPUTE_INTERVAL = 8.0  # s


def _node_sort_key(node_id):
    # GCS always sorts first, so edge slot spacing/anchoring is stable
    return (0, "") if node_id == "GCS" else (1, str(node_id))


def build_mst(nodes: dict):
    """
    nodes: {id: np.ndarray position}, should include 'GCS'.
    Returns list of edges [(id_a, id_b), ...] forming a minimum spanning
    tree (Prim's algorithm -- fine for the small n, ~<= 25, we ever see).
    """
    ids = list(nodes.keys())
    if len(ids) <= 1:
        return []
    in_tree = {ids[0]}
    edges = []
    remaining = set(ids[1:])
    while remaining:
        best = None
        best_d = float("inf")
        for a in in_tree:
            for b in remaining:
                d = float(np.linalg.norm(nodes[a] - nodes[b]))
                if d < best_d:
                    best_d = d
                    best = (a, b)
        a, b = best
        edges.append((a, b))
        in_tree.add(b)
        remaining.discard(b)
    return edges


def _edge_slots(pos_a: np.ndarray, pos_b: np.ndarray):
    """Relay slot positions along one MST edge, spaced at SAFE_HOP_DIST
    (Swarm Relays' Proposition 1 applied per-edge instead of per-target)."""
    vec = pos_b - pos_a
    dist = float(np.linalg.norm(vec))
    if dist <= SAFE_HOP_DIST:
        return []
    n_slots = int(np.ceil(dist / SAFE_HOP_DIST)) - 1
    if n_slots <= 0:
        return []
    direction = vec / dist
    return [pos_a + direction * SAFE_HOP_DIST * (i + 1) for i in range(n_slots)]


class RelayManager:
    def __init__(self):
        self.slot_uav = {}        # (edge_key, slot_idx) -> uav_id
        self.shortage_since = {}  # (edge_key, slot_idx) -> t first seen unfilled
        self.events = []          # (t, event_str) log for report §10 metrics
        self.last_mst_edges = []  # for diagnostics / visualisation
        self._current_edge_ids = []   # list of (id_a, id_b) node-id pairs
        self._edges_computed_at = None
        self._last_owner_ids = frozenset()

    # -----------------------------------------------------------------
    def _active_owners(self, uavs):
        # Include AWAITING_REPORT: a UAV that finished collecting data but
        # hasn't delivered it yet still needs the backbone to reach it --
        # dropping it the instant collection ends (this was the original
        # bug) strands the data with no chance to report at all.
        return [u for u in uavs if u.alive and u.role == UAVRole.SURVEYOR
                and u.state in (UAVState.TRANSIT, UAVState.SURVEYING,
                                 UAVState.AWAITING_REPORT)]

    def update(self, uavs, gcs_pos, t, pois):
        uav_by_id = {u.id: u for u in uavs}
        owners = self._active_owners(uavs)
        nodes = {"GCS": gcs_pos}
        for o in owners:
            # Use the owner's FINAL destination, not its live (moving)
            # position, whenever that destination is already fixed and
            # known (i.e. it's still travelling toward the PoI it was
            # assigned -- o.target is set once, at auction time, and
            # doesn't change until arrival). Using the live position here
            # was a real bug: it made every relay slot a moving target for
            # the whole transit, so relays wasted time chasing an
            # intermediate point instead of beelining to where the chain
            # actually needs to end up. Once the owner has arrived
            # (SURVEYING / AWAITING_REPORT), target == pos anyway, so this
            # is a no-op in that regime.
            nodes[o.id] = o.target if o.target is not None else o.pos

        owner_ids = frozenset(nodes.keys())
        topology_stale = (self._edges_computed_at is None or
                           t - self._edges_computed_at >= MST_RECOMPUTE_INTERVAL)
        owner_set_changed = owner_ids != self._last_owner_ids
        if topology_stale or owner_set_changed:
            self._current_edge_ids = build_mst(nodes)
            self._edges_computed_at = t
            self._last_owner_ids = owner_ids
        # else: keep the same edge (node-id pair) structure, but positions
        # below are still re-read fresh from `nodes` each call, so slots
        # smoothly track their endpoints as owners move.
        edges = [(a, b) for (a, b) in self._current_edge_ids
                 if a in nodes and b in nodes]
        self.last_mst_edges = [(nodes[a], nodes[b]) for a, b in edges]

        # collect this round's slot positions, keyed stably by edge + index
        wanted = {}  # (edge_key, i) -> position
        for a, b in edges:
            a2, b2 = sorted((a, b), key=_node_sort_key)
            edge_key = (a2, b2)
            for i, pos in enumerate(_edge_slots(nodes[a2], nodes[b2])):
                wanted[(edge_key, i)] = pos

        # release slots that no longer exist in this round's backbone
        for key in list(self.slot_uav.keys()):
            if key not in wanted:
                self._free_relay(uav_by_id.get(self.slot_uav[key]), t)
                del self.slot_uav[key]
                self.shortage_since.pop(key, None)

        # Fill trunk (near-GCS) slots before far branch-tip slots. When the
        # fleet can't supply every slot at once, near-GCS slots are worth
        # filling first: they're shared by every downstream branch of the
        # tree, so completing them unblocks the most connectivity per
        # relay used, and they're also the cheapest/fastest to reach.
        # Without this, slot-fill order was arbitrary dict-insertion
        # order, which had no reason to prefer the structurally useful
        # slots when the fleet was contested.
        fill_order = sorted(wanted.items(),
                             key=lambda kv: float(np.linalg.norm(kv[1] - gcs_pos)))

        # fill / update remaining slots
        for key, pos in fill_order:
            rid = self.slot_uav.get(key)
            relay_uav = uav_by_id.get(rid) if rid is not None else None
            if relay_uav is not None and (not relay_uav.alive or
                                           relay_uav.role != UAVRole.RELAY):
                relay_uav = None
                self.slot_uav.pop(key, None)

            if relay_uav is None:
                picked = self._pick_free_uav(uavs, pos, gcs_pos, t)
                if picked is not None:
                    picked.role = UAVRole.RELAY
                    picked.assigned_relay_slot = key
                    picked.set_target(pos)
                    picked.state = UAVState.TRANSIT
                    self.slot_uav[key] = picked.id
                    self.shortage_since.pop(key, None)
                    self.events.append((t, f"relay_assigned uav={picked.id} slot={key}"))
                else:
                    if key not in self.shortage_since:
                        self.shortage_since[key] = t
                        self.events.append((t, f"relay_shortage slot={key}"))
                    elif t - self.shortage_since[key] > RELAY_SHORTAGE_DROP_TIME:
                        self._resolve_persistent_shortage(key, edges, nodes, owners,
                                                            pois, t)
                        self.shortage_since.pop(key, None)
            else:
                relay_uav.set_target(pos)
                if relay_uav.state == UAVState.TRANSIT and \
                   relay_uav.distance_to(pos) < 4.0:
                    relay_uav.state = UAVState.RELAYING

    # -----------------------------------------------------------------
    def _resolve_persistent_shortage(self, key, edges, nodes, owners, pois, t):
        """A relay slot has been unfillable for too long. Find which
        surveyor(s) it was supporting (anyone cut off from GCS if this
        edge were removed) and release their tasks back to PENDING --
        mirrors Ponda et al. 2012's Prune-Task-Space, deterministically."""
        edge_key, _slot_idx = key
        a, b = edge_key
        adj = {n: set() for n in nodes}
        for ea, eb in edges:
            if {ea, eb} == {a, b}:
                continue  # remove the edge this shortage lives on
            adj[ea].add(eb)
            adj[eb].add(ea)
        reached = {"GCS"}
        q = deque(["GCS"])
        while q:
            u = q.popleft()
            for v in adj[u]:
                if v not in reached:
                    reached.add(v)
                    q.append(v)
        cut_off_owner_ids = [o.id for o in owners if o.id not in reached]
        for oid in cut_off_owner_ids:
            uav = next((o for o in owners if o.id == oid), None)
            if uav is None or uav.assigned_task is None:
                continue
            poi = pois[uav.assigned_task]
            if poi.status != PoIStatus.SURVEYED:
                poi.status = PoIStatus.PENDING
                poi.assigned_uav = None
            uav.assigned_task = None
            uav.role = UAVRole.NONE
            uav.target = None
            uav.state = UAVState.IDLE
            self.events.append((t, f"task_dropped_relay_shortage uav={oid} "
                                    f"poi={poi.id}"))

    def chain_ready_for_owner(self, owner_id, uavs) -> bool:
        """True if every relay slot on the edge(s) directly connecting
        `owner_id` into the backbone is filled by a UAV that has actually
        ARRIVED at its slot (state RELAYING), not merely assigned and
        still travelling there. Used to pace a surveyor's final approach:
        no point letting it complete arrival (and start the 10s report
        clock) if the last stretch of its own chain hasn't physically
        gotten there yet -- see sim.py's _update_surveyor_pacing.
        Conservative by construction: only checks edges touching the
        owner directly, not the whole path to GCS, so it can say "ready"
        slightly before the entire chain is truly live; that's an
        acceptable approximation given the trunk-first fill order already
        prioritises getting the GCS-side of the tree solid early.
        """
        uav_by_id = {u.id: u for u in uavs}
        touching_edges = [(a, b) for (a, b) in self._current_edge_ids
                           if owner_id in (a, b)]
        if not touching_edges:
            return True  # no relays needed for this owner at all
        for a, b in touching_edges:
            a2, b2 = sorted((a, b), key=_node_sort_key)
            edge_key = (a2, b2)
            for (ek, _idx), uid in self.slot_uav.items():
                if ek != edge_key:
                    continue
                u = uav_by_id.get(uid)
                if u is None or not u.alive or u.state != UAVState.RELAYING:
                    return False
        return True

    def _pick_free_uav(self, uavs, slot_pos, gcs_pos, t):
        """Same deadline-feasibility principle as the task auction
        (allocation.py's _deadline_feasible): don't commit a UAV to a
        relay slot late in the mission if it couldn't still fly home
        afterwards. Without this, a UAV pulled into relay duty near the
        45-min mark could get stranded exactly like an unfeasible task
        assignment would."""
        candidates = [u for u in uavs if u.alive and u.state == UAVState.IDLE
                      and u.role == UAVRole.NONE]
        feasible = []
        for u in candidates:
            outbound = u.distance_to(slot_pos) / cfg.UAV_SPEED
            inbound = float(np.linalg.norm(gcs_pos - slot_pos)) / cfg.UAV_SPEED
            if t + outbound + inbound + cfg.RTH_TIME_SAFETY_MARGIN <= cfg.MAX_SIM_TIME:
                feasible.append(u)
        if not feasible:
            return None
        feasible.sort(key=lambda u: u.distance_to(slot_pos))
        return feasible[0]

    def _free_relay(self, uav, t):
        if uav is None:
            return
        uav.role = UAVRole.NONE
        uav.assigned_relay_slot = None
        uav.target = None
        uav.state = UAVState.IDLE
        self.events.append((t, f"relay_released uav={uav.id}"))
