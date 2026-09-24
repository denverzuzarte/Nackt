#!/usr/bin/env python3
"""
Scripted scenarios beyond the baseline random-PoI run:

  1. Relay failure  -- kill the first UAV to become a relay, part-way
     into the mission. Measures how long the backbone takes to heal.
  2. Surveyor failure -- kill the first UAV to become a surveyor while
     it's still in transit. Measures how long until the orphaned PoI
     gets picked up by someone else.
  3. Double failure -- both of the above in one run, as a stress test.
  4. Pop-up high-priority PoI -- inject a priority-3 PoI partway through
     the mission and check whether/how the swarm reprioritizes it.

Each scenario runs a throwaway "scout" pass first to find *when* the
event of interest naturally happens for that seed (e.g. when the first
relay gets assigned), then a real run with the failure/pop-up scripted
at that exact moment -- this avoids the uninteresting case of failing
an idle UAV that isn't doing anything yet.

Usage:
    python3 scenarios.py --seed 1
    python3 scenarios.py --seeds 1 2 3 4 5 --scenario relay_failure
"""
import argparse
import numpy as np
from core.sim import Simulation
from core.entities import UAVState, UAVRole, PoIStatus
from core import config as cfg


# ---------------------------------------------------------------------
# Scouting: find a meaningful moment for an event, using a throwaway run
# ---------------------------------------------------------------------
def find_first_relay(seed, after_t=100.0, min_connectivity=5):
    """Find a relay to fail that's actually part of a working network,
    not just the first one to arrive.

    Two refinements over the first version, both found by checking
    results rather than trusting them:
    1. An earlier version picked the very first UAV to reach RELAYING
       state, which often happened before the rest of its own chain had
       arrived -- so overall connectivity was still 0 at the failure
       moment, making "recovery time" measure recovery from nothing.
       Fixed by requiring a real connectivity threshold first.
    2. That fix alone still produced one seed with an anomalous
       baseline of 28 (the ENTIRE fleet) at t=100 -- not genuine
       operational backbone connectivity, but an incidental mesh formed
       by idle UAVs still clustered near their shared spawn/landing
       ring early in the mission (all mutually close together, some
       bridging to GCS). Fixed by also requiring at least one UAV is
       actively SURVEYING -- i.e. the mission has genuinely gotten
       underway, not just launched.
    """
    sim = Simulation(seed=seed)
    while not sim.done:
        sim.step()
        if sim.t < after_t:
            continue
        n_conn = sum(1 for u in sim.uavs if u.alive and u.connected_to_gcs)
        if n_conn < min_connectivity:
            continue
        surveying = any(u.alive and u.state == UAVState.SURVEYING for u in sim.uavs)
        if not surveying:
            continue
        relaying = [u for u in sim.uavs if u.alive and u.role == UAVRole.RELAY
                    and u.state == UAVState.RELAYING]
        if relaying:
            return relaying[0].id, sim.t
    return None, None


def find_first_surveyor_transit(seed, after_t=0.0):
    sim = Simulation(seed=seed)
    while not sim.done:
        sim.step()
        if sim.t < after_t:
            continue
        for u in sim.uavs:
            if u.alive and u.role == UAVRole.SURVEYOR and u.state == UAVState.TRANSIT:
                return u.id, u.assigned_task, sim.t
    return None, None, None


# ---------------------------------------------------------------------
# Scenario 1: relay failure
# ---------------------------------------------------------------------
def _connectivity_recovery(sim, fail_t):
    """
    How this metric evolved, and why -- worth knowing before changing
    it again:

    v1 compared post-failure connectivity to a single instantaneous
    pre-failure reading, guarded by a variable that was never actually
    assigned -- so whenever that reading happened to be 0, the
    "recovered" check (n_conn >= 0) was trivially true on the very next
    step. Fixed by reading from the logged step series instead of
    fragile inline tracking.

    v2 fixed that, but still compared to a single historical reading --
    and one seed had a real (not spurious) baseline of 28, the ENTIRE
    fleet momentarily mutually connected. Averaging over a 20s window
    before the failure didn't help either: connectivity had genuinely
    been sustained near 28 for that whole window, not just a one-step
    spike. "Recovery" to an unrepresentative historical peak isn't a
    meaningful question regardless of how it's smoothed.

    v3 (this version) drops the historical-baseline idea entirely and
    asks a demand-relative question instead: is n_connected currently
    enough for CURRENT operational need (active surveyors + relays at
    that same instant)? This is exactly the definition metrics.py's
    regular (non-scenario) recovery-time calculation already uses, so
    scenario-specific and background recovery numbers are now
    consistent with each other, and it sidesteps the whole "what
    counts as baseline" problem: there's no baseline to be an outlier.
    """
    rows = [r for r in sim.metrics.step_rows if r["t"] >= fail_t]
    if not rows:
        return None
    for r in rows:
        active_demand = r["n_surveyors"] + r["n_relays"]
        deficient = active_demand > 0 and r["n_connected"] < active_demand
        if not deficient:
            return round(r["t"] - fail_t, 1)
    return None  # stayed deficient (relative to demand) for the rest of the mission


def run_relay_failure_scenario(seed, verbose=False):
    relay_uid, fail_t = find_first_relay(seed)
    if relay_uid is None:
        return {"seed": seed, "scenario": "relay_failure", "skipped": "no relay found"}

    sim = Simulation(seed=seed, failure_schedule=[(fail_t, relay_uid)])
    summary = sim.run(verbose=False)
    recovery_dt = _connectivity_recovery(sim, fail_t)
    # informational only, not used in the recovery calc itself (see
    # _connectivity_recovery's docstring for why it's no longer the
    # basis of the recovery-time measurement)
    pre_rows = [r for r in sim.metrics.step_rows if r["t"] < fail_t]
    connectivity_at_failure = pre_rows[-1]["n_connected"] if pre_rows else None

    result = {
        "seed": seed, "scenario": "relay_failure",
        "failed_uav": relay_uid, "fail_t": round(fail_t, 1),
        "connectivity_at_failure": connectivity_at_failure,
        "recovery_time_s": recovery_dt,
        "mission_completion_pct": summary["mission_completion_rate_pct"],
        "pois_reported": summary["pois_reported"],
        "collision_count": summary["collision_count"],
        "geofence_violations": summary["geofence_violations"],
        "charge_violations": summary["charge_violations"],
    }
    if verbose:
        print(result)
    return result


# ---------------------------------------------------------------------
# Scenario 2: surveyor failure (mid-transit, task orphaned)
# ---------------------------------------------------------------------
def run_surveyor_failure_scenario(seed, verbose=False):
    surv_uid, poi_id, fail_t = find_first_surveyor_transit(seed)
    if surv_uid is None:
        return {"seed": seed, "scenario": "surveyor_failure", "skipped": "no surveyor found"}

    sim = Simulation(seed=seed, failure_schedule=[(fail_t, surv_uid)])
    reassigned_t = None
    while not sim.done:
        sim.step()
        poi = sim.pois[poi_id]
        if reassigned_t is None and poi.status in (PoIStatus.ASSIGNED, PoIStatus.SURVEYED):
            # someone (not the dead UAV) picked it up again
            if poi.assigned_uav != surv_uid and poi.assigned_uav is not None:
                reassigned_t = sim.t

    summary = sim.metrics.summarise(sim.uavs, sim.pois, sim.relay_mgr, cfg, sim.t)
    poi = sim.pois[poi_id]
    result = {
        "seed": seed, "scenario": "surveyor_failure",
        "failed_uav": surv_uid, "orphaned_poi": poi_id, "fail_t": round(fail_t, 1),
        "reassignment_delay_s": round(reassigned_t - fail_t, 1) if reassigned_t else None,
        "orphaned_poi_final_status": poi.status.name,
        "orphaned_poi_reported": poi.report_time is not None,
        "mission_completion_pct": summary["mission_completion_rate_pct"],
        "pois_reported": summary["pois_reported"],
        "collision_count": summary["collision_count"],
    }
    if verbose:
        print(result)
    return result


# ---------------------------------------------------------------------
# Scenario 3: double failure (relay + surveyor, stress test)
# ---------------------------------------------------------------------
def run_double_failure_scenario(seed, verbose=False):
    relay_uid, relay_fail_t = find_first_relay(seed)
    surv_uid, poi_id, surv_fail_t = find_first_surveyor_transit(seed)
    schedule = []
    if relay_uid is not None:
        schedule.append((relay_fail_t, relay_uid))
    if surv_uid is not None and surv_uid != relay_uid:
        schedule.append((surv_fail_t, surv_uid))
    if not schedule:
        return {"seed": seed, "scenario": "double_failure", "skipped": "no candidates found"}

    sim = Simulation(seed=seed, failure_schedule=schedule)
    summary = sim.run(verbose=False)
    result = {
        "seed": seed, "scenario": "double_failure",
        "failures": schedule,
        "mission_completion_pct": summary["mission_completion_rate_pct"],
        "pois_reported": summary["pois_reported"],
        "collision_count": summary["collision_count"],
        "geofence_violations": summary["geofence_violations"],
        "charge_violations": summary["charge_violations"],
    }
    if verbose:
        print(result)
    return result


# ---------------------------------------------------------------------
# Scenario 4: pop-up high-priority PoI mid-mission
# ---------------------------------------------------------------------
def run_popup_priority_scenario(seed, popup_t=600.0, verbose=False):
    # place the pop-up PoI far from GCS so it's a genuinely costly
    # commitment, not a trivial nearby pickup
    popup_pos = (850.0, 150.0)
    sim = Simulation(seed=seed, popup_pois=[(popup_pos, 3, popup_t)])
    popup_id = cfg.N_POIS  # first PoI appended after the base N_POIS

    # track: how long after reveal does someone bid on it? does it
    # preempt an in-progress lower-priority task?
    bid_t = None
    while not sim.done:
        sim.step()
        poi = sim.pois[popup_id]
        if bid_t is None and poi.status != PoIStatus.PENDING:
            bid_t = sim.t

    summary = sim.metrics.summarise(sim.uavs, sim.pois, sim.relay_mgr, cfg, sim.t)
    poi = sim.pois[popup_id]
    result = {
        "seed": seed, "scenario": "popup_priority",
        "popup_reveal_t": popup_t,
        "pickup_delay_s": round(bid_t - popup_t, 1) if bid_t else None,
        "popup_final_status": poi.status.name,
        "popup_reported": poi.report_time is not None,
        "popup_report_latency_s": poi.report_latency,
        "mission_completion_pct": summary["mission_completion_rate_pct"],
        "priority_weighted_score": summary["priority_weighted_score"],
        "priority_weighted_score_max": summary["priority_weighted_score_max"],
    }
    if verbose:
        print(result)
    return result


def find_contention_moment(seed, n_uavs, min_pending=2):
    """Scout for a moment where a real backlog exists (>= min_pending
    revealed-but-unassigned PoIs simultaneously) under a given fleet
    size, rather than guessing a fixed time. A first attempt at the A/B
    priority test used a fixed popup_t=300 and found ZERO backlog in
    every case even at n_uavs=8 -- 10 PoIs spread over a 1500s reveal
    window rarely queue up even with a small fleet, so contention has
    to be deliberately located, not assumed."""
    sim = Simulation(seed=seed, n_uavs=n_uavs)
    while not sim.done:
        sim.step()
        n_pending = sum(1 for p in sim.pois
                         if p.status == PoIStatus.PENDING and p.is_revealed(sim.t))
        if n_pending >= min_pending:
            return sim.t, n_pending
    return None, 0


def run_priority_ab_test(seed, n_uavs=8, popup_t=None, verbose=False):
    """
    Does priority weighting actually change behavior, or has it just
    never been tested under real contention? With the default 28-UAV
    fleet and only 10 PoIs, priority_weighted_score has hit its max in
    every scenario we've run -- there's essentially always a free UAV
    available, so nothing ever has to choose between PoIs. That's not
    evidence the mechanism works, just that it's never been exercised.

    This runs the SAME seed, SAME pop-up position and reveal time,
    under a deliberately constrained fleet (so genuine backlog exists),
    twice -- once with the pop-up at priority 3 (critical) and once at
    priority 1 (normal) -- and compares how quickly each gets picked up.
    If priority is doing real work, the priority-3 version should be
    serviced meaningfully faster under identical contention.
    """
    popup_pos = (850.0, 150.0)

    if popup_t is None:
        popup_t, found_pending = find_contention_moment(seed, n_uavs, min_pending=2)
        if popup_t is None:
            return {"seed": seed, "scenario": "priority_ab_test",
                    "skipped": f"no contention moment found at n_uavs={n_uavs}"}

    def run_with_priority(prio):
        sim = Simulation(seed=seed, n_uavs=n_uavs,
                          popup_pois=[(popup_pos, prio, popup_t)])
        popup_id = cfg.N_POIS
        pending_at_reveal = None
        bid_t = None
        while not sim.done:
            sim.step()
            if pending_at_reveal is None and sim.t >= popup_t:
                pending_at_reveal = sum(
                    1 for p in sim.pois if p.id != popup_id
                    and p.status == PoIStatus.PENDING and p.is_revealed(sim.t))
            poi = sim.pois[popup_id]
            if bid_t is None and poi.status != PoIStatus.PENDING:
                bid_t = sim.t
        summary = sim.metrics.summarise(sim.uavs, sim.pois, sim.relay_mgr, cfg, sim.t)
        return {
            "pickup_delay_s": round(bid_t - popup_t, 1) if bid_t else None,
            "other_pending_at_reveal": pending_at_reveal,
            "mission_completion_pct": summary["mission_completion_rate_pct"],
            "priority_weighted_score": summary["priority_weighted_score"],
            "priority_weighted_score_max": summary["priority_weighted_score_max"],
        }

    high = run_with_priority(3)
    low = run_with_priority(1)
    result = {
        "seed": seed, "scenario": "priority_ab_test", "n_uavs": n_uavs,
        "popup_t": popup_t,
        "high_priority_run": high,
        "low_priority_run": low,
        "priority_mattered": (
            high["pickup_delay_s"] is not None and low["pickup_delay_s"] is not None
            and high["pickup_delay_s"] < low["pickup_delay_s"]
        ),
    }
    if verbose:
        print(result)
    return result


SCENARIOS = {
    "relay_failure": run_relay_failure_scenario,
    "surveyor_failure": run_surveyor_failure_scenario,
    "double_failure": run_double_failure_scenario,
    "popup_priority": run_popup_priority_scenario,
}


def aggregate(results, numeric_fields, bool_fields=()):
    """Summarise a list of per-seed scenario result dicts: mean/min/max
    for numeric fields (skipping None), and success-rate for bool
    fields. Skipped/failed scenario runs (a "skipped" key present) are
    excluded from numeric stats but counted separately."""
    valid = [r for r in results if "skipped" not in r]
    skipped = [r for r in results if "skipped" in r]
    out = {"n_seeds": len(results), "n_valid": len(valid), "n_skipped": len(skipped)}
    for f in numeric_fields:
        vals = [r[f] for r in valid if r.get(f) is not None]
        if vals:
            out[f] = {"mean": round(float(np.mean(vals)), 2),
                       "min": round(float(np.min(vals)), 2),
                       "max": round(float(np.max(vals)), 2),
                       "n": len(vals)}
        else:
            out[f] = None
    for f in bool_fields:
        vals = [r[f] for r in valid if f in r]
        out[f + "_rate"] = round(sum(vals) / len(vals), 2) if vals else None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    ap.add_argument("--scenario", type=str, choices=list(SCENARIOS) + ["all"], default="all")
    args = ap.parse_args()

    scenario_names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    for name in scenario_names:
        print(f"\n=== {name} ===")
        fn = SCENARIOS[name]
        for seed in args.seeds:
            r = fn(seed, verbose=True)


if __name__ == "__main__":
    main()
