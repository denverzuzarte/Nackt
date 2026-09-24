#!/usr/bin/env python3
"""
Entry point for running the UAV swarm disaster-response simulation.

Usage:
    python3 run_demo.py                      # single run, seed=1, verbose
    python3 run_demo.py --seed 7              # a different scenario
    python3 run_demo.py --seeds 1 2 3 4 5      # run several seeds, print a summary table
    python3 run_demo.py --n-uavs 20 --n-pois 10
    python3 run_demo.py --quiet                # suppress the periodic progress line
    python3 run_demo.py --save-csv out         # dump per-step + summary data to out_*.csv/json

See README.md for what every field in the summary means and for the
current state of the project (what's implemented, what's known to be
imperfect, what's next).
"""
import argparse
import json
from core.sim import Simulation


def run_one(seed, n_uavs, n_pois, verbose, save_csv_prefix=None):
    sim = Simulation(seed=seed, n_uavs=n_uavs, n_pois=n_pois)
    summary = sim.run(verbose=verbose)
    if save_csv_prefix:
        prefix = f"{save_csv_prefix}_seed{seed}"
        sim.metrics.dump(prefix)
        sim.metrics.dump_summary(summary, prefix)
        print(f"  -> wrote {prefix}_steps.csv and {prefix}_summary.json")
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1, help="single scenario seed (default: 1)")
    ap.add_argument("--seeds", type=int, nargs="+", default=None,
                     help="run several seeds instead of one, e.g. --seeds 1 2 3 4 5")
    ap.add_argument("--n-uavs", type=int, default=None, help="override fleet size (default: config.N_UAVS=28)")
    ap.add_argument("--n-pois", type=int, default=None, help="override PoI count (default: config.N_POIS=10)")
    ap.add_argument("--quiet", action="store_true", help="suppress the periodic progress line")
    ap.add_argument("--save-csv", type=str, default=None,
                     help="prefix for output CSV/JSON files, e.g. --save-csv results/run")
    args = ap.parse_args()

    seeds = args.seeds if args.seeds is not None else [args.seed]

    all_summaries = []
    for sd in seeds:
        print(f"\n=== seed={sd} ===")
        s = run_one(sd, args.n_uavs, args.n_pois, verbose=not args.quiet,
                     save_csv_prefix=args.save_csv)
        all_summaries.append((sd, s))
        print(json.dumps(s, indent=2))

    if len(all_summaries) > 1:
        print("\n=== summary across seeds ===")
        header = f"{'seed':6s}{'completion%':12s}{'reported':10s}{'lat_mean_s':12s}{'collisions':11s}"
        print(header)
        for sd, s in all_summaries:
            print(f"{sd:<6d}{s['mission_completion_rate_pct']:<12}"
                  f"{str(s['pois_reported'])+'/10':<10s}"
                  f"{str(s['report_latency_mean_s']):<12s}"
                  f"{s['collision_count']:<11d}")


if __name__ == "__main__":
    main()
