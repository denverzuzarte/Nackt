#!/usr/bin/env python3
"""
Generate the static figures for the written report (§9.2 "PoC results:
trajectories, connectivity, coverage over time").

Usage:
    python3 make_report_figures.py --seed 1 --out-dir figs
"""
import argparse
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.lines import Line2D
from matplotlib.collections import LineCollection

from core.sim import Simulation
from core import config as cfg
from core.entities import PoIStatus

# Segment color depends on STATE, not just role: role alone conflates a
# UAV's brief SURVEYOR excursion with the much longer idle time it spent
# with role=NONE waiting at the landing pad -- an earlier version of this
# figure picked one dominant color per UAV's whole lifetime by total step
# count, which mostly picked "idle grey" even for UAVs with substantial
# real activity, since idle time dominates raw step counts. Coloring each
# segment by the state active *at that moment* instead gives an accurate
# picture: idle/charging segments are short and stationary (landing pad),
# so they barely show as lines, while active segments show in their real
# color regardless of how much idle time surrounds them.
SEGMENT_COLOR = {
    "SURVEYOR_ACTIVE": "#2E7BD6",   # TRANSIT/SURVEYING/AWAITING_REPORT as SURVEYOR
    "RELAY_ACTIVE": "#D69A2E",       # TRANSIT/RELAYING as RELAY
    "RTH": "#B98CFF",
    "IDLE": "#3A4552",
}
PRIO_COLOR = {1: "#2E7BD6", 2: "#D69A2E", 3: "#D6392E"}


def segment_category(state, role):
    if state == "RTH":
        return "RTH"
    if role == "SURVEYOR":
        return "SURVEYOR_ACTIVE"
    if role == "RELAY":
        return "RELAY_ACTIVE"
    return "IDLE"


def make_trajectory_figure(sim, out_path):
    fig, ax = plt.subplots(figsize=(9, 8.5))
    ax.set_facecolor("#0B0F14")
    fig.patch.set_facecolor("#0B0F14")

    lo, hi = cfg.GEOFENCE_MIN, cfg.GEOFENCE_MAX
    ax.set_xlim(lo[0] - 20, hi[0] + 20)
    ax.set_ylim(lo[1] - 20, hi[1] + 20)
    ax.set_aspect("equal")

    # geofence + operational area
    ax.add_patch(Rectangle(lo, hi[0] - lo[0], hi[1] - lo[1], fill=False,
                            linestyle=":", edgecolor="#4C596B", linewidth=1))
    op_lo, op_hi = cfg.OP_AREA_MIN, cfg.OP_AREA_MAX
    ax.add_patch(Rectangle(op_lo, op_hi[0] - op_lo[0], op_hi[1] - op_lo[1],
                            fill=False, linestyle="--", edgecolor="#7E8CA0", linewidth=1.2))

    # trajectories per UAV, each segment colored by the state/role active
    # at that moment (see segment_category). Subsample every 4th recorded
    # step (2s of sim time) -- plenty smooth for a static overview and
    # keeps the LineCollection a manageable size (28 UAVs x ~1300 points
    # instead of x ~5300).
    n_uavs = len(sim.uavs)
    frames = sim.history[::4]
    per_uav = {i: {"pts": [], "state": [], "role": []} for i in range(n_uavs)}
    for frame in frames:
        for (uid, pos, state, role, alive, conn, batt) in frame["uavs"]:
            if alive:
                per_uav[uid]["pts"].append(pos)
                per_uav[uid]["state"].append(state)
                per_uav[uid]["role"].append(role)

    segments_by_cat = {cat: [] for cat in SEGMENT_COLOR}
    for uid in range(n_uavs):
        pts = per_uav[uid]["pts"]
        states = per_uav[uid]["state"]
        roles = per_uav[uid]["role"]
        for i in range(len(pts) - 1):
            cat = segment_category(states[i], roles[i])
            segments_by_cat[cat].append([pts[i], pts[i + 1]])

    # draw idle first (background), active categories on top, most subtle first
    for cat in ["IDLE", "RTH", "RELAY_ACTIVE", "SURVEYOR_ACTIVE"]:
        segs = segments_by_cat[cat]
        if not segs:
            continue
        alpha = 0.15 if cat == "IDLE" else 0.55
        lw = 0.5 if cat == "IDLE" else 0.9
        lc = LineCollection(segs, colors=SEGMENT_COLOR[cat], alpha=alpha, linewidths=lw)
        ax.add_collection(lc)

    # GCS
    gcs = cfg.GCS_POS
    ax.plot(*gcs, marker="D", markersize=10, color="#2E7BD6", zorder=5)
    ax.annotate("GCS", gcs, textcoords="offset points", xytext=(10, 6),
                color="#E7EDF3", fontsize=10, fontweight="bold")
    ax.add_patch(Circle(gcs, cfg.COMM_RANGE, fill=False, edgecolor="#2E7BD6",
                         alpha=0.2, linewidth=1))

    # PoIs, final status
    for p in sim.pois:
        color = PRIO_COLOR.get(p.priority, "#7E8CA0")
        filled = p.status == PoIStatus.SURVEYED
        ax.plot(*p.pos, marker="o", markersize=9 + p.priority * 2,
                markerfacecolor=("#35D28A" if filled else "none"),
                markeredgecolor=color, markeredgewidth=1.8, zorder=6)
        ax.annotate(f"P{p.id}", p.pos, textcoords="offset points", xytext=(7, 5),
                     color="#7E8CA0", fontsize=8)

    ax.set_title("UAV trajectories and final PoI status", color="#E7EDF3", fontsize=13)
    ax.tick_params(colors="#7E8CA0")
    for spine in ax.spines.values():
        spine.set_color("#232B36")

    legend_elems = [
        Line2D([0], [0], color=SEGMENT_COLOR["SURVEYOR_ACTIVE"], lw=2, label="Surveyor transit"),
        Line2D([0], [0], color=SEGMENT_COLOR["RELAY_ACTIVE"], lw=2, label="Relay transit"),
        Line2D([0], [0], color=SEGMENT_COLOR["RTH"], lw=2, label="Returning to GCS"),
        Line2D([0], [0], color=SEGMENT_COLOR["IDLE"], lw=2, label="Idle / charging"),
        Line2D([0], [0], marker="D", color="w", markerfacecolor="#2E7BD6",
               label="GCS (+ comm range)", markersize=8, linestyle="None"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#35D28A",
               markeredgecolor="#35D28A", label="PoI surveyed", markersize=9, linestyle="None"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="none",
               markeredgecolor="#7E8CA0", label="PoI not surveyed", markersize=9, linestyle="None"),
    ]
    leg = ax.legend(handles=legend_elems, loc="lower right", facecolor="#131A22",
                     edgecolor="#232B36", fontsize=8.5, labelcolor="#E7EDF3")

    fig.tight_layout()
    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)


def make_timeseries_figure(sim, out_path):
    rows = sim.metrics.step_rows
    t = [r["t"] for r in rows]
    n_surveyed = [r["n_pois_surveyed"] for r in rows]
    n_connected = [r["n_connected"] for r in rows]
    n_relays = [r["n_relays"] for r in rows]
    n_surveyors = [r["n_surveyors"] for r in rows]

    fig, axes = plt.subplots(2, 1, figsize=(9.5, 6.5), sharex=True)
    for ax in axes:
        ax.set_facecolor("#0B0F14")
    fig.patch.set_facecolor("#0B0F14")

    ax0 = axes[0]
    ax0.plot(t, n_surveyed, color="#35D28A", linewidth=1.8, label="PoIs surveyed")
    ax0.axhline(len(sim.pois), color="#4C596B", linestyle=":", linewidth=1)
    ax0.set_ylabel("PoIs surveyed", color="#E7EDF3")
    ax0.set_title("Coverage over time", color="#E7EDF3", fontsize=12)
    ax0.tick_params(colors="#7E8CA0")

    ax1 = axes[1]
    ax1.plot(t, n_connected, color="#2E7BD6", linewidth=1.6, label="UAVs connected to GCS")
    ax1.plot(t, n_relays, color="#D69A2E", linewidth=1.2, linestyle="--", label="Active relays")
    ax1.plot(t, n_surveyors, color="#8A93A3", linewidth=1.2, linestyle=":", label="Active surveyors")
    ax1.set_ylabel("UAV count", color="#E7EDF3")
    ax1.set_xlabel("Mission time (s)", color="#E7EDF3")
    ax1.set_title("Connectivity and role allocation over time", color="#E7EDF3", fontsize=12)
    ax1.tick_params(colors="#7E8CA0")
    ax1.legend(loc="upper right", facecolor="#131A22", edgecolor="#232B36",
               fontsize=8.5, labelcolor="#E7EDF3")

    for ax in axes:
        for spine in ax.spines.values():
            spine.set_color("#232B36")
        ax.grid(color="#16202B", linewidth=0.6)

    fig.tight_layout()
    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out-dir", type=str, default="figs")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    sim = Simulation(seed=args.seed)
    summary = sim.run(verbose=True)

    traj_path = os.path.join(args.out_dir, f"trajectories_seed{args.seed}.png")
    ts_path = os.path.join(args.out_dir, f"coverage_connectivity_seed{args.seed}.png")
    make_trajectory_figure(sim, traj_path)
    make_timeseries_figure(sim, ts_path)
    print(f"\nWrote {traj_path}")
    print(f"Wrote {ts_path}")
    print(f"\nSummary: {summary}")


if __name__ == "__main__":
    main()
