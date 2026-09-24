"""
Task allocation: CBAA-lite.

Full CBBA/CBAA (Choi, Brunet & How 2009) is a distributed auction +
max-consensus protocol run over an inter-agent communication network, with
convergence guaranteed under the diminishing-marginal-gain (DMG) property.

For Stage 1 we implement the *outcome* of a single-assignment auction
(CBAA) each replanning round -- i.e. a centralised simulation of what the
distributed bidding/consensus process converges to -- rather than
message-passing agents. This is stated explicitly as a simplification:
the real CBAA protocol is what would run on each UAV in hardware, and is
noted as such in the report (this stays a fair stand-in because CBAA is
proven to converge to a conflict-free assignment; simulating the
converged result lets us focus dev time on the allocation-connectivity
interface, which is the novel part of this design).

Scoring function (priority-weighted, distance-discounted -- this is the
kind of scoring function CBBA is designed around, see Choi et al. 2009
eq. 11 time-discounted reward):

    score(uav, poi) = PRIORITY_WEIGHTS[poi.priority] / (1 + dist(uav, poi) / D0)
                       * relay_cost_discount(poi, free_uav_count)

The relay_cost_discount term is a lightweight nod to Ponda et al.
2012's coupling of task value and relay cost in a single bid (CBBA with
Relays merges relay tasks directly into the CBBA bid matrix; we don't go
that far here -- see README for why -- but a PoI that is many relay hops
from GCS, bid on when few UAVs are free to become relays, is discounted
so the auction naturally prefers "affordable" PoIs when the fleet is
relay-constrained, rather than only discovering the shortage after the
fact in the relay manager).

Only PENDING, revealed PoIs are auctioned. Only UAVs currently IDLE (free)
place bids. Highest bid wins each PoI; ties broken by UAV id for
determinism.
"""
from __future__ import annotations
import numpy as np
from . import config as cfg
from . import relay as relay_mod
from .entities import UAVState, UAVRole, PoIStatus

D0 = 400.0  # distance-discount normaliser, metres


def relay_cost_discount(gcs_pos: np.ndarray, poi_pos: np.ndarray,
                         free_uav_count: int) -> float:
    """Rough estimate of how many relay hops a PoI would need from GCS,
    normalised against how many free UAVs are currently available to
    supply them. >=1 hop needs -> discount grows as free_uav_count
    shrinks relative to hops_needed. Deliberately crude (straight-line,
    ignores the shared-backbone savings other active surveyors provide)
    -- it only needs to bias the auction, not predict connectivity
    exactly; the relay manager's MST backbone is the source of truth.
    """
    dist = float(np.linalg.norm(poi_pos - gcs_pos))
    hops_needed = max(0, int(np.ceil(dist / relay_mod.SAFE_HOP_DIST)) - 1)
    if hops_needed == 0:
        return 1.0
    return 1.0 / (1.0 + hops_needed / max(free_uav_count, 1))


def score(uav_pos: np.ndarray, poi, gcs_pos: np.ndarray, free_uav_count: int) -> float:
    d = float(np.linalg.norm(uav_pos - poi.pos))
    w = cfg.PRIORITY_WEIGHTS.get(poi.priority, 1.0)
    base = w / (1.0 + d / D0)
    return base * relay_cost_discount(gcs_pos, poi.pos, free_uav_count)


def _deadline_feasible(uav_pos, poi_pos, gcs_pos, t: float) -> bool:
    """
    COMPETITION SPEC: all UAVs must land by the 45-min mark. Bidding is
    the only point where a UAV *commits* to going somewhere -- the
    per-step RTH check further downstream can only react to a bad
    commitment after the fact, it can't undo one. So the feasibility
    check belongs here: refuse to bid on a PoI at all if flying there,
    surveying it, and flying home afterwards wouldn't fit in the time
    remaining. This was a real bug we found by letting the simulation
    run to its actual conclusion instead of stopping early once PoIs
    were "surveyed": UAVs were being sent on brand-new, far assignments
    with only minutes left in the mission and never made it home.
    """
    outbound = float(np.linalg.norm(poi_pos - uav_pos)) / cfg.UAV_SPEED
    survey = cfg.TASK_SERVICE_TIME
    inbound = float(np.linalg.norm(gcs_pos - poi_pos)) / cfg.UAV_SPEED
    total_needed = outbound + survey + inbound + cfg.RTH_TIME_SAFETY_MARGIN
    return (t + total_needed) <= cfg.MAX_SIM_TIME


def run_auction(uavs: list, pois: list, t: float, gcs_pos: np.ndarray):
    """
    Runs one round of the CBAA-lite auction. Mutates uav.assigned_task,
    uav.target, uav.state and poi.status/assigned_uav in place.

    Returns list of (uav_id, poi_id) new assignments made this round, for
    logging (this feeds the "relay reallocations" / allocation-event log).
    """
    free_uavs = [u for u in uavs if u.alive and u.state == UAVState.IDLE]
    pending_pois = [p for p in pois if p.status == PoIStatus.PENDING and p.is_revealed(t)]

    if not free_uavs or not pending_pois:
        return []

    free_uav_count = len(free_uavs)
    # Build bid matrix -- infeasible (uav, poi) pairs (couldn't survey AND
    # get home in time) are simply never bid on, rather than being scored
    # low; a PoI with no feasible bidder this round just stays PENDING.
    bids = {}  # (uav_id, poi_id) -> score
    for u in free_uavs:
        for p in pending_pois:
            if not _deadline_feasible(u.pos, p.pos, gcs_pos, t):
                continue
            bids[(u.id, p.id)] = score(u.pos, p, gcs_pos, free_uav_count)

    new_assignments = []
    remaining_uavs = {u.id: u for u in free_uavs}
    remaining_pois = {p.id: p for p in pending_pois}

    # Greedy winner-take-all auction: repeatedly assign the single highest
    # remaining bid, remove that uav and poi, repeat. This is the standard
    # sequential-greedy result CBAA/CBBA is proven to reproduce under DMG
    # scoring (Choi et al. 2009, Sec. V-A "Sequential Greedy Algorithm").
    while remaining_uavs and remaining_pois:
        best = None
        best_score = -1.0
        for uid in remaining_uavs:
            for pid in remaining_pois:
                s = bids.get((uid, pid), -1.0)  # missing = deadline-infeasible
                if s > best_score or (s == best_score and best is not None and
                                       (uid, pid) < best):
                    best_score = s
                    best = (uid, pid)
        if best is None:
            break
        uid, pid = best
        uav = remaining_uavs.pop(uid)
        poi = remaining_pois.pop(pid)

        uav.assigned_task = poi.id
        uav.set_target(poi.pos)
        uav.state = UAVState.TRANSIT
        uav.role = UAVRole.SURVEYOR
        poi.status = PoIStatus.ASSIGNED
        poi.assigned_uav = uav.id
        new_assignments.append((uid, pid))

    return new_assignments
