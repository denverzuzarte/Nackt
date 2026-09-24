#!/usr/bin/env python3
"""
Run a simulation and export a downsampled, compact history for the
interactive HTML visualization (viz/build_viz.py consumes this).

Usage:
    python3 export_history.py --seed 1 --out history.json --frame-every 4
"""
import argparse
import json
import re
from core.sim import Simulation
from core import config as cfg

_SLOT_RE = re.compile(r"slot=\(\('?([\w.\-]+)'?, '?([\w.\-]+)'?\), (\d+)\)")


def clean_event_text(text):
    """Raw event strings carry Python tuple reprs for slot keys, e.g.
    "slot=(('GCS', 5), 0)". Reformat as "slot=GCS-5:0" for readability
    in the UI's event ticker."""
    return _SLOT_RE.sub(lambda m: f"slot={m.group(1)}-{m.group(2)}:{m.group(3)}", text)

STATE_CODE = {
    "IDLE": 0, "TRANSIT": 1, "SURVEYING": 2, "AWAITING_REPORT": 3,
    "RELAYING": 4, "RTH": 5, "CHARGING": 6, "FAILED": 7,
}
ROLE_CODE = {"NONE": 0, "SURVEYOR": 1, "RELAY": 2}
POI_STATUS_CODE = {"PENDING": 0, "ASSIGNED": 1, "SURVEYED": 2}


def r1(x):
    return round(float(x), 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--n-uavs", type=int, default=None)
    ap.add_argument("--n-pois", type=int, default=None)
    ap.add_argument("--out", type=str, default="history.json")
    ap.add_argument("--frame-every", type=int, default=4,
                     help="keep 1 in every N simulated steps (DT=0.5s default -> "
                          "frame-every=4 means one frame per 2s of sim time)")
    args = ap.parse_args()

    sim = Simulation(seed=args.seed, n_uavs=args.n_uavs, n_pois=args.n_pois)
    summary = sim.run(verbose=True)

    frames = []
    for i, frame in enumerate(sim.history):
        if i % args.frame_every != 0 and i != len(sim.history) - 1:
            continue
        uavs = [
            [uid, r1(pos[0]), r1(pos[1]), STATE_CODE[state], ROLE_CODE[role],
             int(alive), int(conn), int(round(batt * 100))]
            for (uid, pos, state, role, alive, conn, batt) in frame["uavs"]
        ]
        pois = [
            [pid, r1(pos[0]), r1(pos[1]), POI_STATUS_CODE[status], prio]
            for (pid, pos, status, prio) in frame["pois"]
        ]
        backbone = [
            [[r1(a[0]), r1(a[1])], [r1(b[0]), r1(b[1])]]
            for (a, b) in frame["backbone"]
        ]
        frames.append([r1(frame["t"]), uavs, pois, backbone])

    # Unified event ticker: relay-manager events plus synthesized
    # detection/report events, sorted by time, for a live log in the UI.
    events = [[r1(t), clean_event_text(text)] for (t, text) in sim.relay_mgr.events]
    for p in sim.pois:
        if p.detection_time is not None:
            events.append([r1(p.detection_time), f"poi_detected poi={p.id} prio={p.priority}"])
        if p.report_time is not None:
            lat = p.report_latency
            flag = " LATE" if lat is not None and lat > cfg.MAX_REPORT_LATENCY else ""
            events.append([r1(p.report_time), f"poi_reported poi={p.id} latency={lat:.1f}s{flag}"])
    events.sort(key=lambda e: e[0])

    poi_events = [
        {
            "id": p.id, "priority": p.priority,
            "reveal_time": r1(p.reveal_time),
            "detection_time": r1(p.detection_time) if p.detection_time is not None else None,
            "report_time": r1(p.report_time) if p.report_time is not None else None,
        }
        for p in sim.pois
    ]

    out = {
        "meta": {
            "gcs": [r1(cfg.GCS_POS[0]), r1(cfg.GCS_POS[1])],
            "op_area": [list(cfg.OP_AREA_MIN), list(cfg.OP_AREA_MAX)],
            "geofence": [list(cfg.GEOFENCE_MIN), list(cfg.GEOFENCE_MAX)],
            "comm_range": cfg.COMM_RANGE,
            "n_uavs": len(sim.uavs),
            "n_pois": len(sim.pois),
            "seed": args.seed,
            "frame_dt": r1(cfg.DT * args.frame_every),
            "mission_deadline": cfg.MAX_SIM_TIME,
            "state_legend": {v: k for k, v in STATE_CODE.items()},
            "role_legend": {v: k for k, v in ROLE_CODE.items()},
            "poi_status_legend": {v: k for k, v in POI_STATUS_CODE.items()},
        },
        "summary": summary,
        "poi_events": poi_events,
        "events": events,
        "frames": frames,
    }

    with open(args.out, "w") as f:
        json.dump(out, f, separators=(",", ":"))

    import os
    size_mb = os.path.getsize(args.out) / (1024 * 1024)
    print(f"\nWrote {args.out}: {len(frames)} frames, {size_mb:.2f} MB")


if __name__ == "__main__":
    main()
