"""
Main simulation driver.
"""
from __future__ import annotations
import numpy as np
from . import config as cfg
from . import comm
from . import orca
from . import relay as relay_mod
from .entities import UAV, PoI, UAVState, UAVRole, PoIStatus
from .allocation import run_auction
from .relay import RelayManager
from .metrics import MetricsLogger

GCS_POS = np.array(cfg.GCS_POS, dtype=float)

# Landing-pad ring: UAVs returning home cannot all be sent to the exact
# same point, since the 20 m minimum-separation requirement makes it
# geometrically impossible for more than one of them to actually get
# within the "arrived" threshold of a single point at once -- the rest
# would orbit forever, unable to land, and drain their battery to zero
# while circling (this was a real bug found during hardening: several
# UAVs did exactly this). Instead each UAV gets its own fixed slot on a
# ring around the GCS, sized so that even with the whole fleet parked
# simultaneously (the "rush hour" at the 45-min deadline, see the
# GEOFENCE_MIN comment in config.py), adjacent slots are still >= the
# required separation apart.
def landing_pad_pos(uav_id: int, n_uavs: int) -> np.ndarray:
    n = max(n_uavs, 2)
    # Ring radius sized so adjacent slots clear not just the raw 20 m
    # separation requirement but ORCA's own exclusion diameter (2 x
    # UAV_RADIUS) with real headroom. Spacing this only 10 m above the
    # bare minimum (an earlier version of this fix) still let a target
    # slot sit right at the edge of a neighbouring parked UAV's exclusion
    # zone -- ORCA then holds the UAV a few metres short of "arrived"
    # forever (observed: stuck at 4.3-4.5 m against a 4.0 m threshold,
    # for the rest of the mission, draining its battery to zero). The
    # extra margin here is deliberately generous since the cost of a
    # larger ring is negligible (the geofence has room) but the cost of
    # under-sizing it is a UAV that can never land.
    min_chord = 2 * cfg.UAV_RADIUS + 15.0
    radius = max(min_chord / (2 * np.sin(np.pi / n)), 40.0)
    angle = 2 * np.pi * uav_id / n
    return GCS_POS + radius * np.array([np.cos(angle), np.sin(angle)])


class Simulation:
    def __init__(self, n_uavs=None, n_pois=None, seed=None,
                 failure_schedule=None, popup_pois=None):
        self.rng = np.random.default_rng(seed if seed is not None else cfg.RANDOM_SEED)
        n_uavs = n_uavs or cfg.N_UAVS
        n_pois = n_pois or cfg.N_POIS

        # Spawn on the same landing-pad ring used for RTH returns (see
        # landing_pad_pos) so spacing is guaranteed safe from t=0 and
        # consistent with where UAVs will later return to.
        self.uavs = [UAV(i, landing_pad_pos(i, n_uavs)) for i in range(n_uavs)]
        self.pois = self._make_pois(n_pois, popup_pois or [])
        self.failure_schedule = failure_schedule or []  # list of (t, uav_id)

        self.relay_mgr = RelayManager()
        self.metrics = MetricsLogger()
        self.t = 0.0
        self.history = []  # list of dicts per step, for animation replay
        self.done = False

    def _make_pois(self, n_pois, popup_specs):
        pois = []
        lo = np.array(cfg.OP_AREA_MIN)
        hi = np.array(cfg.OP_AREA_MAX)
        for i in range(n_pois):
            pos = self.rng.uniform(lo, hi)
            priority = int(self.rng.choice([1, 1, 1, 2, 2, 3]))
            reveal_t = float(self.rng.uniform(*cfg.POI_REVEAL_WINDOW))
            pois.append(PoI(i, pos, priority=priority, reveal_time=reveal_t))
        # scripted pop-up high-priority PoIs, e.g. [(pos, priority, reveal_time)]
        for j, (pos, priority, reveal_t) in enumerate(popup_specs):
            pois.append(PoI(n_pois + j, np.array(pos, dtype=float),
                             priority=priority, reveal_time=reveal_t))
        return pois

    # -----------------------------------------------------------------
    def _apply_failures(self):
        """
        BUG FIXED HERE: a UAV that dies while holding assigned_task used
        to strand its PoI forever. UAV.kill() only touches UAV-local
        state (alive/state/role/target) -- it never releases the PoI,
        because a UAV method shouldn't reach into the PoI list. But
        nothing else did that release either: run_auction() only looks
        at PENDING PoIs, so a PoI left at ASSIGNED (pointing at a now-dead
        UAV id) could never be picked up by anyone else again, silently,
        for the rest of the mission. This exact release-on-departure
        pattern already existed in _send_to_rth() for deliberate
        recalls; failures just never got the same treatment. Mirrored
        here for the involuntary case.
        """
        for (ft, uid) in self.failure_schedule:
            if abs(ft - self.t) < cfg.DT / 2:
                uav = self.uavs[uid]
                if uav.alive:
                    if uav.assigned_task is not None:
                        poi = self.pois[uav.assigned_task]
                        if poi.status != PoIStatus.SURVEYED:
                            poi.status = PoIStatus.PENDING
                            poi.assigned_uav = None
                        uav.assigned_task = None
                    uav.kill()
                    self.relay_mgr.events.append((self.t, f"failure uav={uid}"))

    def _time_to_home(self, u):
        return u.distance_to(GCS_POS) / cfg.UAV_SPEED

    def _battery_and_state_machine(self):
        for u in self.uavs:
            if not u.alive:
                continue

            # trigger RTH: whichever comes first -- battery running low, or
            # not enough mission time left to fly home (COMPETITION SPEC:
            # all UAVs must land by the 45-min mark)
            time_left = cfg.MAX_SIM_TIME - self.t
            must_rth = (
                u.battery <= cfg.BATTERY_RTH_THRESHOLD or
                time_left <= self._time_to_home(u) + cfg.RTH_TIME_SAFETY_MARGIN
            )
            if must_rth and u.state not in (UAVState.RTH, UAVState.CHARGING):
                self._send_to_rth(u)

            # arrival handling. RTH uses a looser threshold than PoI/relay
            # arrival on purpose -- see config.py's RTH_ARRIVAL_THRESHOLD
            # comment for why (ORCA has no path planning, so a UAV a few
            # metres from a congested landing slot can otherwise stall
            # indefinitely; one was directly observed stuck for 500+s).
            if u.target is not None and u.state == UAVState.TRANSIT and \
               u.role == UAVRole.SURVEYOR and u.distance_to(u.target) < 4.0:
                u.state = UAVState.SURVEYING
                u.survey_timer = cfg.TASK_SERVICE_TIME
                poi = self.pois[u.assigned_task]
                if poi.detection_time is None:
                    poi.detection_time = self.t
                    poi.detecting_uav_id = u.id
            elif u.target is not None and u.state == UAVState.RTH and \
                    u.distance_to(u.target) < cfg.RTH_ARRIVAL_THRESHOLD:
                u.state = UAVState.CHARGING
                u.target = None
                u.rth_start_time = None
            elif u.state == UAVState.RTH and u.target is not None:
                # stuck-landing fallback: force it down where it is rather
                # than let it stall near its slot for the rest of the
                # mission (defense in depth -- the loosened threshold
                # above should resolve most cases without needing this)
                if u.rth_start_time is None:
                    u.rth_start_time = self.t
                elif (self.t - u.rth_start_time > cfg.RTH_STUCK_TIMEOUT and
                      u.distance_to(u.target) < cfg.RTH_STUCK_RADIUS):
                    self.relay_mgr.events.append(
                        (self.t, f"rth_stuck_force_land uav={u.id} "
                                 f"dist={u.distance_to(u.target):.1f}"))
                    u.state = UAVState.CHARGING
                    u.target = None
                    u.rth_start_time = None

            if u.state == UAVState.SURVEYING:
                u.survey_timer -= cfg.DT
                if u.survey_timer <= 0:
                    self._finish_data_collection(u)

            if u.state == UAVState.CHARGING:
                u.recharge(cfg.DT)
                if u.battery >= 0.95:
                    u.state = UAVState.IDLE
                    u.role = UAVRole.NONE

            u.drain_battery(cfg.DT)
            if u.battery <= 0.0 and u.state != UAVState.CHARGING:
                self.metrics.log_charge_violation(self.t, u.id)

    def _send_to_rth(self, u):
        # release whatever this UAV was doing so the task/slot frees up
        if u.assigned_task is not None:
            poi = self.pois[u.assigned_task]
            if poi.status != PoIStatus.SURVEYED:
                poi.status = PoIStatus.PENDING
                poi.assigned_uav = None
            u.assigned_task = None
        u.role = UAVRole.NONE
        u.assigned_relay_slot = None
        u.state = UAVState.RTH
        u.set_target(landing_pad_pos(u.id, len(self.uavs)))
        self.relay_mgr.events.append((self.t, f"rth uav={u.id}"))

    def _finish_data_collection(self, u):
        """Physical data collection at the PoI is done (this is what
        counts toward mission-completion / coverage). The UAV does NOT
        free up yet -- it holds position as an "owner" the relay backbone
        keeps trying to reach, until its report actually gets through
        (or it's forced to give up via RTH). See _update_poi_reporting
        and _release_after_report."""
        poi = self.pois[u.assigned_task]
        poi.status = PoIStatus.SURVEYED
        poi.survey_complete_time = self.t
        poi.was_connected_during_survey = u.connected_to_gcs
        u.state = UAVState.AWAITING_REPORT
        # role stays SURVEYOR, target stays at the PoI position: this is
        # what keeps it counted as an active "owner" for the relay
        # backbone (RelayManager._active_owners) until it reports.

    def _release_after_report(self, u):
        u.assigned_task = None
        u.role = UAVRole.NONE
        u.target = None
        u.state = UAVState.IDLE

    def _update_connectivity(self):
        nodes = {'GCS': GCS_POS}
        for u in self.uavs:
            if u.alive:
                nodes[u.id] = u.pos
        dist, _adj = comm.connectivity_to_gcs(nodes)
        for u in self.uavs:
            if not u.alive:
                u.connected_to_gcs = False
                u.path_hops_to_gcs = None
                continue
            hops = dist.get(u.id)
            u.connected_to_gcs = hops is not None
            u.path_hops_to_gcs = hops
            if u.connected_to_gcs:
                u.time_connected += cfg.DT
            else:
                u.time_disconnected += cfg.DT

    def _update_poi_reporting(self):
        """COMPETITION SPEC: max 10 s between PoI detection and reporting
        to GCS. A PoI is "reported" the first moment after detection that
        its detecting UAV has a live multi-hop path to the GCS."""
        for p in self.pois:
            if p.detection_time is not None and p.report_time is None:
                uav = self.uavs[p.detecting_uav_id]
                if uav.alive and uav.connected_to_gcs:
                    p.report_time = self.t
                    if uav.state == UAVState.AWAITING_REPORT:
                        self._release_after_report(uav)

    def _check_safety(self):
        alive = [u for u in self.uavs if u.alive]
        for a in range(len(alive)):
            for b in range(a + 1, len(alive)):
                d = alive[a].distance_to(alive[b].pos)
                if d < cfg.UAV_MIN_SEPARATION:
                    self.metrics.log_collision(self.t, alive[a].id, alive[b].id, d)
        lo = cfg.GEOFENCE_MIN
        hi = cfg.GEOFENCE_MAX
        for u in alive:
            x, y = u.pos
            if not (lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1]):
                self.metrics.log_geofence_violation(self.t, u.id, u.pos)

    def _predecessor_position(self, u):
        """The node this UAV should not outrun: for a relay, the previous
        (lower-index, closer-to-GCS) slot on the same backbone edge; for
        the surveyor at the front of a chain, the highest-index (closest
        to it) relay slot on its edge. None on either end resolves to GCS
        itself (chain-of-one, or no inner relays needed for this hop)."""
        if u.role == UAVRole.RELAY and u.assigned_relay_slot is not None:
            edge_key, idx = u.assigned_relay_slot
            if idx == 0:
                return GCS_POS
            prev_uid = self.relay_mgr.slot_uav.get((edge_key, idx - 1))
            if prev_uid is not None and self.uavs[prev_uid].alive:
                return self.uavs[prev_uid].pos
            return None  # inward neighbour not assigned yet -- no info,
                          # don't freeze the outer link on this alone

        if u.role == UAVRole.SURVEYOR and u.state in (
                UAVState.TRANSIT, UAVState.SURVEYING, UAVState.AWAITING_REPORT):
            edge = next((e for e in self.relay_mgr._current_edge_ids
                         if u.id in e), None)
            if edge is None:
                return GCS_POS
            a2, b2 = sorted(edge, key=relay_mod._node_sort_key)
            edge_key = (a2, b2)
            best_idx, best_uid = -1, None
            for (ek, idx), uid in self.relay_mgr.slot_uav.items():
                if ek == edge_key and idx > best_idx:
                    best_idx, best_uid = idx, uid
            if best_uid is not None and self.uavs[best_uid].alive:
                return self.uavs[best_uid].pos
            return GCS_POS  # no relays on this edge -- direct hop to GCS

        return None

    def _throttle_backbone_advance(self):
        """Swarm Relays-style gating (Varadharajan et al. 2020, Eq. 6-7):
        a chain link should not advance far ahead of its inward neighbour,
        so the chain forms progressively outward from GCS instead of every
        link racing independently to its own destination and hoping they
        land in sync (which was the direct cause of the reporting-latency
        violations before this fix: the surveyor consistently arrived
        well before its relay chain).

        Implemented as a SOFT linear ramp rather than the paper's hard
        stop/go switch: full speed while within SAFE_HOP_DIST of the
        predecessor (normal steady-state spacing), ramping down to a
        floor (never fully zero) by BACKBONE_GATE_DIST. A hard 0/full
        cutoff was tried first and caused total gridlock -- with several
        chain links simultaneously sitting near the threshold, each
        freezes waiting on the one behind it, which is itself frozen
        waiting on the one behind *it*, and the whole backbone deadlocks
        permanently (0% mission completion in testing). The soft ramp
        guarantees forward creep as long as the predecessor is making any
        progress at all, which avoids that failure mode while still
        substantially discouraging outrunning the chain."""
        gate_start = relay_mod.SAFE_HOP_DIST
        gate_full = relay_mod.BACKBONE_GATE_DIST
        speed_floor = 0.2 * cfg.UAV_SPEED
        for u in self.uavs:
            if not u.alive:
                u.speed_cap = cfg.UAV_SPEED
                continue
            pred = self._predecessor_position(u)
            if pred is None:
                u.speed_cap = cfg.UAV_SPEED
                continue
            d = u.distance_to(pred)
            if d <= gate_start:
                u.speed_cap = cfg.UAV_SPEED
            elif d >= gate_full:
                u.speed_cap = speed_floor
            else:
                frac = (gate_full - d) / (gate_full - gate_start)
                u.speed_cap = speed_floor + frac * (cfg.UAV_SPEED - speed_floor)

    def _update_surveyor_pacing(self):
        """Don't let a surveyor complete its final approach (and thereby
        start the 10s detection-to-report clock) if the last stretch of
        its own relay chain hasn't physically arrived yet. Without this,
        the surveyor races to the PoI at full speed regardless of whether
        its backbone is ready, which was the single biggest driver of
        report latency we measured: relays dispatched at the same moment
        as the surveyor, but arriving after it because of fleet
        contention or simple bad luck in who got picked for which slot.
        Only holds during the final hop's worth of approach, not the
        whole transit -- holding earlier would just delay detection
        without helping, since the chain isn't expected to be ready yet
        that far out anyway."""
        for u in self.uavs:
            u.hold_for_relay = False
            if not u.alive or u.role != UAVRole.SURVEYOR or u.state != UAVState.TRANSIT:
                u.hold_since = None
                continue
            if u.target is None:
                u.hold_since = None
                continue
            if u.distance_to(u.target) <= relay_mod.SAFE_HOP_DIST and \
               not self.relay_mgr.chain_ready_for_owner(u.id, self.uavs):
                if u.hold_since is None:
                    u.hold_since = self.t
                if self.t - u.hold_since < cfg.SURVEYOR_HOLD_MAX_S:
                    u.hold_for_relay = True
                # else: given up waiting, proceed anyway (accept the
                # latency hit rather than risk never finishing the task)
            else:
                u.hold_since = None

    def _move_all_with_separation(self):
        """Resolve this step's velocities with ORCA (van den Berg et al.
        2011) so UAVs never need to violate the 20 m min-separation
        requirement just to reach their allocation/relay targets, then
        apply them synchronously."""
        new_vel = orca.compute_new_velocities(self.uavs, cfg.DT)
        for u in self.uavs:
            if u.id in new_vel:
                u.apply_velocity(new_vel[u.id], cfg.DT)

    # -----------------------------------------------------------------
    def step(self):
        self._apply_failures()

        if abs(self.t % cfg.REPLAN_INTERVAL) < cfg.DT / 2:
            run_auction(self.uavs, self.pois, self.t, GCS_POS)

        self._battery_and_state_machine()
        self.relay_mgr.update(self.uavs, GCS_POS, self.t, self.pois)
        # NOTE: a backbone-advance throttle (Swarm Relays' Eq. 6-7 idea --
        # don't let a chain link outrun its inward neighbour, applied to
        # EVERY link in a chain, each gated on the one behind it) was
        # tried here to fix reporting latency, both as a hard stop/go
        # switch and as a softened linear ramp. Both made things
        # substantially worse (mission completion dropped to 0% and 70%
        # respectively): with several chain links near the gate threshold
        # simultaneously, each waits on the one behind it, which is
        # itself waiting -- global slowdown/gridlock rather than the
        # intended local synchronisation. Reverted; _predecessor_position
        # and _throttle_backbone_advance above are left in as dead code,
        # not deleted, since they're a real documented dead end worth
        # keeping visible rather than silently discarding.
        #
        # _update_surveyor_pacing (below) is a narrower, different
        # mechanism: only the SURVEYOR itself ever holds (never a relay
        # holding on another relay), and only for its own final hop of
        # approach. No link waits on another link, so the cascading
        # multi-agent gridlock above structurally can't occur here --
        # but given that history, this was verified against gridlock
        # empirically before being trusted, not assumed safe. See README.
        self._update_surveyor_pacing()

        self._move_all_with_separation()

        self._update_connectivity()
        self._update_poi_reporting()
        self._check_safety()

        self.metrics.log_step(self.t, self.uavs, self.pois, None, GCS_POS)
        self._record_history_frame()

        self.t += cfg.DT

        # COMPETITION SPEC: all UAVs must land by the 45-min mark. Stopping
        # the simulation as soon as every PoI is *surveyed* (the original
        # condition) was a real bug: it meant we never actually simulated
        # far enough to observe -- let alone verify -- whether every UAV
        # made it home in time. We now only end early if coverage is done
        # AND every UAV has actually landed; otherwise we run to the
        # mission deadline, which is also the honest way to check the
        # deadline-triggered RTH logic actually works.
        n_surveyed = sum(1 for p in self.pois if p.status == PoIStatus.SURVEYED)
        all_landed = all(
            (not u.alive) or u.state in (UAVState.IDLE, UAVState.CHARGING)
            for u in self.uavs
        )
        if self.t >= cfg.MAX_SIM_TIME:
            self.done = True
        elif n_surveyed == len(self.pois) and all_landed:
            self.done = True

    def _record_history_frame(self):
        self.history.append({
            "t": self.t,
            "uavs": [(u.id, u.pos.copy(), u.state.name, u.role.name, u.alive,
                      u.connected_to_gcs, u.battery) for u in self.uavs],
            "pois": [(p.id, p.pos.copy(), p.status.name, p.priority) for p in self.pois],
            "backbone": list(self.relay_mgr.last_mst_edges),
        })

    def run(self, verbose=False):
        while not self.done:
            self.step()
            if verbose and abs(self.t % 60) < cfg.DT / 2:
                n_surv = sum(1 for p in self.pois if p.status == PoIStatus.SURVEYED)
                n_conn = sum(1 for u in self.uavs if u.alive and u.connected_to_gcs)
                n_alive = sum(1 for u in self.uavs if u.alive)
                print(f"t={self.t:6.0f}s | surveyed {n_surv}/{len(self.pois)} "
                      f"| alive {n_alive}/{len(self.uavs)} | connected {n_conn}")
        return self.metrics.summarise(self.uavs, self.pois, self.relay_mgr,
                                       cfg, self.t)
