"""
Metrics logger. Accumulates per-step and per-event data, then produces the
summary numbers needed for report §10 (Evaluation & Performance Metrics).
"""
from __future__ import annotations
import json
import csv
import numpy as np
from .entities import PoIStatus


class MetricsLogger:
    def __init__(self):
        self.step_rows = []          # per-timestep snapshot rows
        self.collision_events = []   # (t, uav_a, uav_b, dist)
        self.geofence_violations = []  # (t, uav_id, pos)
        self.charge_violations = []  # (t, uav_id)  -- battery hit 0 while alive
        self.start_time = None
        self.end_time = None
        self.min_pairwise_distance_ever = None
        self.packet_events = []  # tracks EVERY pair's
            # distance each step, not just threshold violations -- see
            # log_pairwise_distance / _min_separation for why this is
            # tracked separately from collision_events.

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

    def log_packet_tx(self, t, uav_id, pkt_type, delivered, latency_ms, hops, path_losses, snrs):
        self.packet_events.append({
            "t": t,
            "uav_id": uav_id,
            "pkt_type": pkt_type,
            "delivered": delivered,
            "latency_ms": latency_ms,
            "hops": hops,
            "path_losses": path_losses,
            "snrs": snrs,
        })

    def log_collision(self, t, id_a, id_b, dist):
        self.collision_events.append((t, id_a, id_b, dist))

    def log_pairwise_distance(self, dist):
        if self.min_pairwise_distance_ever is None or dist < self.min_pairwise_distance_ever:
            self.min_pairwise_distance_ever = dist

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

        # connectivity availability: fraction of (active-role-timesteps,
        # i.e. surveyors AND relays -- a relay whose own path home breaks
        # is exactly as much a connectivity failure as a surveyor's, so
        # both count as "demand") that were connected. Approximates
        # packet delivery ratio for Stage 1 (no actual packet-level
        # simulation). BUG FIX: this used to count only n_surveyors as
        # demand, while _compute_recovery_times' "deficient" check (right
        # below) already used n_surveyors + n_relays -- two different,
        # silently inconsistent definitions of "active demand" for what
        # is supposed to be the same underlying question. Unified on the
        # broader (recovery-time's) definition.
        #
        # COMMUNICATION DOWNTIME (report Sec. 10 metric): total seconds
        # where active demand existed but wasn't fully met -- the direct
        # complement of connectivity availability, in absolute time
        # rather than a percentage.
        connected_active_steps = 0
        total_active_steps = 0
        downtime_steps = 0
        for row in self.step_rows:
            active = row["n_surveyors"] + row["n_relays"]
            if active > 0:
                total_active_steps += active
                connected_active_steps += min(row["n_connected"], active)
                if row["n_connected"] < active:
                    downtime_steps += 1
        connectivity_availability = (connected_active_steps / total_active_steps
                                      if total_active_steps else None)
        communication_downtime_s = downtime_steps * config_module.DT

        # recovery time: for each disconnection->reconnection transition of
        # any UAV that has an active role, measure duration
        recovery_times = self._compute_recovery_times()

        relay_reallocations = sum(1 for (t, e) in relay_manager.events
                                   if e.startswith("relay_assigned"))
        relay_preemptions = sum(1 for (t, e) in relay_manager.events
                                 if e.startswith("relay_preempted"))
        relay_shortages = sum(1 for (t, e) in relay_manager.events
                               if e.startswith("relay_shortage"))
        tasks_dropped_for_relay_shortage = sum(
            1 for (t, e) in relay_manager.events
            if e.startswith("task_dropped_relay_shortage"))

        # NETWORK RECONFIGURATION EFFICIENCY (report Sec. 10 metric,
        # previously undefined -- see relay.py's topology_recompute event
        # for the reasoning). Each event carries reused=<n> total=<n>;
        # efficiency for that event is reused/total, and the overall
        # metric is the mean across every recompute in the run. 100%
        # would mean every recompute kept the entire existing tree (only
        # ever grew/shrunk at the edges, never rebuilt); 0% means every
        # recompute discarded the whole prior backbone.
        reconfig_ratios = []
        for (_t, e) in relay_manager.events:
            if not e.startswith("topology_recompute"):
                continue
            parts = dict(tok.split("=") for tok in e.split()[1:])
            reused = int(parts["reused"])
            total = int(parts["total"])
            reconfig_ratios.append(reused / total if total else 1.0)
        network_reconfig_efficiency_pct = (
            round(100 * sum(reconfig_ratios) / len(reconfig_ratios), 1)
            if reconfig_ratios else None)

        # SECTION 10 & 5.1/5.2: Communication Packet & Latency Metrics
        total_packets = len(self.packet_events)
        delivered_packets = [p for p in self.packet_events if p["delivered"]]
        connected_attempts = [p for p in self.packet_events if p["hops"] is not None]

        pdr_mission_pct = round(100.0 * len(delivered_packets) / total_packets, 1) if total_packets else 100.0
        pdr_connected_pct = round(100.0 * len(delivered_packets) / len(connected_attempts), 1) if connected_attempts else 100.0

        pkt_latencies = [p["latency_ms"] for p in delivered_packets if p["latency_ms"] is not None]
        packet_latency_mean_ms = round(float(np.mean(pkt_latencies)), 2) if pkt_latencies else None
        packet_latency_p95_ms = round(float(np.percentile(pkt_latencies, 95)), 2) if pkt_latencies else None
        packet_latency_max_ms = round(float(np.max(pkt_latencies)), 2) if pkt_latencies else None
        packet_jitter_ms = round(float(np.std(pkt_latencies)), 2) if len(pkt_latencies) > 1 else 0.0

        all_pl = [pl for p in delivered_packets for pl in p["path_losses"]]
        all_snr = [snr for p in delivered_packets for snr in p["snrs"]]
        mean_path_loss_db = round(float(np.mean(all_pl)), 2) if all_pl else None
        mean_snr_db = round(float(np.mean(all_snr)), 2) if all_snr else None

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
            "communication_downtime_s": round(communication_downtime_s, 1),
            "packet_delivery_ratio_pct": pdr_mission_pct,
            "packet_delivery_ratio_connected_pct": pdr_connected_pct,
            "packet_latency_mean_ms": packet_latency_mean_ms,
            "packet_latency_p95_ms": packet_latency_p95_ms,
            "packet_latency_max_ms": packet_latency_max_ms,
            "packet_jitter_ms": packet_jitter_ms,
            "mean_path_loss_db": mean_path_loss_db,
            "mean_snr_db": mean_snr_db,
            "relay_reallocations": relay_reallocations,
            "relay_preemptions": relay_preemptions,
            "relay_shortages_logged": relay_shortages,
            "tasks_dropped_for_relay_shortage": tasks_dropped_for_relay_shortage,
            "network_reconfig_efficiency_pct": network_reconfig_efficiency_pct,
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
        # The smallest distance ever observed between any two alive UAVs
        # over the whole run -- NOT derived from collision_events, which
        # only logs threshold VIOLATIONS and would report None on every
        # clean run (see log_pairwise_distance's call site for why that
        # was a real bug). round()'d for readability; None only if the
        # fleet genuinely never had 2+ alive UAVs simultaneously (e.g. a
        # 1-UAV run), which doesn't happen in practice here.
        if self.min_pairwise_distance_ever is None:
            return None
        return round(self.min_pairwise_distance_ever, 2)

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
