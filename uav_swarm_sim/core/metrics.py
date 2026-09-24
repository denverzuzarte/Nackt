"""
Metrics logger. Accumulates per-step and per-event data, then produces the
summary numbers needed for report §10 (Evaluation & Performance Metrics).
"""
from __future__ import annotations
import json
import csv
from .entities import PoIStatus


class MetricsLogger:
    def __init__(self):
        self.step_rows = []          # per-timestep snapshot rows
        self.collision_events = []   # (t, uav_a, uav_b, dist)
        self.geofence_violations = []  # (t, uav_id, pos)
        self.charge_violations = []  # (t, uav_id)  -- battery hit 0 while alive
        self.start_time = None
        self.end_time = None

    def log_step(self, t, uavs, pois, relay_events_this_step, gcs_pos):
        n_connected = sum(1 for u in uavs if u.alive and u.connected_to_gcs)
        n_alive = sum(1 for u in uavs if u.alive)
        n_surveyed = sum(1 for p in pois if p.status == PoIStatus.SURVEYED)
        n_relays = sum(1 for u in uavs if u.alive and u.role.name == "RELAY")
        n_surveyors = sum(1 for u in uavs if u.alive and u.role.name == "SURVEYOR")
        self.step_rows.append({
            "t": t,
            "n_alive": n_alive,
            "n_connected": n_connected,
            "n_pois_surveyed": n_surveyed,
            "n_pois_total": len(pois),
            "n_relays": n_relays,
            "n_surveyors": n_surveyors,
        })

    def log_collision(self, t, id_a, id_b, dist):
        self.collision_events.append((t, id_a, id_b, dist))

    def log_geofence_violation(self, t, uid, pos):
        self.geofence_violations.append((t, uid, tuple(pos)))

    def log_charge_violation(self, t, uid):
        self.charge_violations.append((t, uid))

    # -----------------------------------------------------------------
    def summarise(self, uavs, pois, relay_manager, config_module, mission_end_t):
        n_pois = len(pois)
        n_surveyed = sum(1 for p in pois if p.status == PoIStatus.SURVEYED)
        completion_rate = n_surveyed / n_pois if n_pois else 0.0

        priority_weighted_score = sum(
            config_module.PRIORITY_WEIGHTS.get(p.priority, 1.0)
            for p in pois if p.status == PoIStatus.SURVEYED
        )
        max_priority_score = sum(
            config_module.PRIORITY_WEIGHTS.get(p.priority, 1.0) for p in pois
        )

        # connectivity availability: fraction of (surveyor-active-timesteps)
        # that were connected, i.e. "was the data able to get home while it
        # mattered". Approximates packet delivery ratio for Stage 1 (no
        # actual packet-level simulation).
        connected_active_steps = 0
        total_active_steps = 0
        for row in self.step_rows:
            if row["n_surveyors"] > 0:
                total_active_steps += row["n_surveyors"]
                connected_active_steps += min(row["n_connected"], row["n_surveyors"])
        connectivity_availability = (connected_active_steps / total_active_steps
                                      if total_active_steps else None)

        # recovery time: for each disconnection->reconnection transition of
        # any UAV that has an active role, measure duration
        recovery_times = self._compute_recovery_times()

        relay_reallocations = sum(1 for (t, e) in relay_manager.events
                                   if e.startswith("relay_assigned"))
        relay_shortages = sum(1 for (t, e) in relay_manager.events
                               if e.startswith("relay_shortage"))
        tasks_dropped_for_relay_shortage = sum(
            1 for (t, e) in relay_manager.events
            if e.startswith("task_dropped_relay_shortage"))

        # COMPETITION SPEC: max 10 s between PoI detection and reporting.
        latencies = [p.report_latency for p in pois if p.report_latency is not None]
        n_detected = sum(1 for p in pois if p.detection_time is not None)
        n_reported = sum(1 for p in pois if p.report_time is not None)
        n_late = sum(1 for lat in latencies if lat > config_module.MAX_REPORT_LATENCY)
        n_never_reported = n_detected - n_reported

        summary = {
            "mission_completion_rate_pct": round(100 * completion_rate, 1),
            "mission_completion_time_s": mission_end_t,
            "priority_weighted_score": priority_weighted_score,
            "priority_weighted_score_max": max_priority_score,
            "connectivity_availability_pct": (
                round(100 * connectivity_availability, 1)
                if connectivity_availability is not None else None
            ),
            "relay_reallocations": relay_reallocations,
            "relay_shortages_logged": relay_shortages,
            "tasks_dropped_for_relay_shortage": tasks_dropped_for_relay_shortage,
            "pois_detected": n_detected,
            "pois_reported": n_reported,
            "pois_never_reported": n_never_reported,
            "report_latency_mean_s": (round(sum(latencies) / len(latencies), 2)
                                       if latencies else None),
            "report_latency_max_s": (round(max(latencies), 2) if latencies else None),
            "report_latency_violations_gt_10s": n_late,
            "recovery_times_s": recovery_times,
            "mean_recovery_time_s": (
                round(sum(recovery_times) / len(recovery_times), 1)
                if recovery_times else None
            ),
            "collision_count": len(self.collision_events),
            "min_separation_observed_m": self._min_separation(),
            "geofence_violations": len(self.geofence_violations),
            "charge_violations": len(self.charge_violations),
            "n_uavs_total": len(uavs),
            "n_uavs_alive_end": sum(1 for u in uavs if u.alive),
        }
        return summary

    def _min_separation(self):
        if not self.collision_events:
            return None
        return min(d for (_, _, _, d) in self.collision_events)

    def _compute_recovery_times(self):
        """Walk the per-step connected-count series; whenever n_connected
        drops below n_surveyors+n_relays (i.e. someone active lost the
        link) and later recovers, record the gap. Coarse but adequate for
        Stage 1 reporting."""
        times = []
        was_deficient = False
        deficit_start = None
        for row in self.step_rows:
            active = row["n_surveyors"] + row["n_relays"]
            deficient = row["n_connected"] < active and active > 0
            if deficient and not was_deficient:
                deficit_start = row["t"]
                was_deficient = True
            elif not deficient and was_deficient:
                times.append(row["t"] - deficit_start)
                was_deficient = False
        return times

    # -----------------------------------------------------------------
    def dump(self, out_prefix):
        with open(f"{out_prefix}_steps.csv", "w", newline="") as f:
            if self.step_rows:
                writer = csv.DictWriter(f, fieldnames=list(self.step_rows[0].keys()))
                writer.writeheader()
                writer.writerows(self.step_rows)

    def dump_summary(self, summary, out_prefix):
        with open(f"{out_prefix}_summary.json", "w") as f:
            json.dump(summary, f, indent=2)
