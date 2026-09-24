# UAV Swarm Disaster-Response Simulation — Stage 1 Working Build

A Python simulation of a UAV swarm that surveys randomly-spawning Points
of Interest (PoIs) in a disaster area while maintaining a resilient
multi-hop relay link back to a Ground Control Station (GCS), built
against the competition's Round 1 mission constraints.

## Quick start

```bash
cd uav_swarm_sim
python3 run_demo.py                          # single run, seed=1, verbose progress
python3 run_demo.py --quiet                    # same, no progress line
python3 run_demo.py --seed 7                    # different random scenario
python3 run_demo.py --seeds 1 2 3 4 5            # run several seeds, print a comparison table
python3 run_demo.py --n-uavs 20 --n-pois 10       # override fleet/PoI counts
python3 run_demo.py --save-csv results/run        # also dump per-step CSV + summary JSON
```

Requires Python 3.10+, `numpy`, `scipy` (only used transitively; not a
hard dependency of the core loop). No other packages needed. Tested on
Python 3.12.

```bash
pip install numpy scipy   # if not already available
```

Each run takes roughly 10-20 seconds wall-clock for one seed (the
mission is up to 2700 simulated seconds at 0.5s steps, with a fleet of
up to 28 UAVs running ORCA collision avoidance and MST-based relay
planning every step).

## What this is, in one paragraph

Every simulated second: any newly-revealed PoIs get auctioned off to
free UAVs (CBAA-lite); the relay manager builds a shared minimum-spanning
tree connecting the GCS to every currently-active surveyor and staffs
its edges with free UAVs as relays; a surveyor paces its final approach
to wait for its own relay chain to actually arrive before "detecting"
the PoI; every UAV's velocity is resolved through ORCA so the 20m
minimum-separation requirement is never violated; battery drains only in
flight, and every UAV is guaranteed — by a deadline-feasibility check at
assignment time plus a generous reactive safety margin — to return to
the GCS before the 45-minute mission ends.

## Verified against the competition constraints (10 seeds)

| Constraint | Status |
|---|---|
| Mission ≤ 45 min | ✅ |
| Continuous flight ≤ 20 min | ✅ |
| Comm range 100m | ✅ (hard threshold in `core/comm.py`) |
| 1000m×1000m operational area | ✅ |
| Take off from / land at operational center by 45 min | ✅ (verified across 10 seeds — see below) |
| Height ≤ 100m | ⚠️ not modeled (2D sim, single flight layer — a stated simplification, not a violation, just untested) |
| Max speed 5m/s | ✅ (hard cap in ORCA's linear program) |
| Min separation 20m | ✅ — **0 collisions across every seed tested** |
| 10s detection→report latency | ❌ — mean ~59s across 10 seeds (range ~4–170s), some individual detections into the hundreds of seconds. See "Known limitations" below. |
| 10 PoIs, random position + time | ✅ |

## Architecture (see individual files for full design rationale)

```
core/
  config.py      - every tunable parameter, with COMPETITION SPEC vs
                    our-own-choice clearly marked in comments
  entities.py     - UAV and PoI data classes, state machines
  comm.py         - distance-threshold link model, BFS connectivity to GCS
  allocation.py   - CBAA-lite auction (Choi et al. 2009-style scoring,
                    with a deadline-feasibility gate and a relay-cost
                    discount borrowed from Ponda et al. 2012)
  relay.py        - shared MST relay backbone (not one chain per
                    surveyor — see file docstring for why), trunk-first
                    slot filling, shortage-triggered task release
  orca.py         - ORCA collision avoidance (van den Berg et al. 2011),
                    implemented from the paper, not a library
  sim.py          - the main step() loop tying everything together,
                    battery/RTH state machine, surveyor pacing, landing
  metrics.py       - collects everything the report's results table needs
run_demo.py        - CLI entry point
```

## What's implemented vs. simplified (be upfront about this in the report)

**Implemented for real, not stubbed:**
- CBAA-lite task allocation with priority weighting and deadline feasibility
- A shared MST relay backbone (not per-target chains) — a deliberate
  efficiency improvement over the closest prior work (Ponda et al. 2012's
  CBBA-with-Relays creates relays per disconnected task, one gap at a
  time; Swarm Relays gives each target its own independent chain)
- ORCA collision avoidance, implemented from the paper's math, not a
  wrapper around an existing library
- Battery, deadline-aware return-to-home, and a landing-ring fix so the
  whole fleet can physically land simultaneously without gridlocking
  on the 20m separation requirement
- Surveyor pacing to reduce (not eliminate) the detection-to-report
  latency gap

**Explicit, stated simplifications:**
- 2D only — no altitude, no 3D dynamics, single flight layer assumed
  within the 100m ceiling
- Kinematic point-mass UAVs — no attitude, no acceleration limits,
  instantaneous heading changes (bounded in effect by ORCA + the
  separation requirement, but not physically modeled)
- Communication is a hard distance threshold, not an SNR/terrain model
- CBAA-lite simulates the *converged outcome* of the distributed
  CBAA/CBBA protocol each round, rather than actually message-passing
  agents through bid/consensus rounds

## Known limitations (honest, not hidden)

1. **10s reporting latency is not fully met.** Partly a physics floor
   (a single new relay hop takes ~17s minimum to fly into position at
   5m/s over the ~95m safe-hop spacing we use — no reactive scheme can
   beat that without pre-positioning relays before a task is even
   assigned) and partly fleet contention under our chosen fleet size
   (28 UAVs, chosen by sweeping 20/24/28/32 — see `config.py`'s comment
   on `N_UAVS`). Mean latency was cut from ~95s to ~59s across the same
   10 seeds by pacing the surveyor's final approach (see
   `sim.py:_update_surveyor_pacing`), but individual hard cases still
   run into the hundreds of seconds.
2. **A more ambitious latency fix was tried and reverted.** Throttling
   *every* relay link against its inward neighbor (not just the
   surveyor against its whole chain) caused cascading gridlock — mission
   completion dropped to 0% and 70% in testing. The dead code
   (`sim.py:_predecessor_position`, `_throttle_backbone_advance`) is
   left in place, not deleted, as a documented dead end.
3. **No visualization/animation yet.** The simulation records a full
   per-step history (`Simulation.history`) suitable for building one;
   it just hasn't been built.
4. **No failure-injection or pop-up-PoI scenario has been run yet**,
   though the machinery for both already exists: `Simulation(...,
   failure_schedule=[(t, uav_id), ...], popup_pois=[(pos, priority,
   reveal_time), ...])`.
5. Two-layer architecture (allocation and connectivity as separate
   modules talking through the UAV state, not a single unified
   algorithm) was a deliberate choice for debuggability under a tight
   deadline, not a claim that it's the "correct" design — see the
   conversation history for the tradeoffs against a unified
   CBBA-with-Relays-style approach.

## Visualization

Two outputs, both built from the same recorded `Simulation.history`:

**1. Static report figures** (for §9.2's "trajectories, connectivity,
coverage over time"):

```bash
python3 make_report_figures.py --seed 1 --out-dir figs
```

Produces `figs/trajectories_seed1.png` (UAV paths, each segment colored
by the role/state active at that moment — surveyor transit, relay
transit, returning-to-GCS, idle — plus final PoI status) and
`figs/coverage_connectivity_seed1.png` (PoIs surveyed, UAVs connected,
active relays/surveyors, all vs. mission time).

**2. Interactive playback** (mission-control style: map + live
telemetry panel + fleet roster + scrolling event log + play/scrub/speed
controls):

```bash
python3 export_history.py --seed 1 --out history.json --frame-every 4
python3 viz/build_viz.py --data history.json --out visualization.html
```

`visualization.html` is fully self-contained (data embedded, no
external JS dependencies beyond Google Fonts) — open it directly in a
browser, or publish it wherever an artifact/static page can be hosted.
`--frame-every 4` keeps 1 in every 4 simulated steps (one frame per 2s
of sim time); lower it for smoother playback at the cost of a larger
file (a full-resolution export of a ~2650s mission is several MB; 4 is
a reasonable default, tested and verified working).

Both `viz/template.html`'s JS and `export_history.py`'s data pipeline
were verified end-to-end before being trusted: extracted the embedded
script, ran it under a stubbed DOM in Node across every recorded frame
(no runtime errors), and cross-checked the final on-screen telemetry
values against `Simulation.summarise()`'s actual numbers (exact match).
This wasn't just eyeballing the canvas output — see the conversation
history for the harness, if picking this pattern up for future changes.

## Failure-injection and pop-up-priority scenarios (`scenarios.py`)

```bash
python3 scenarios.py --seeds 1 2 3 4 5                     # all scenarios
python3 scenarios.py --seeds 1 2 3 --scenario relay_failure
```

Four scenario types, each using a "scout" pass to find a *meaningful*
moment for the event (failing an idle UAV, or testing priority with no
real contention, would be uninteresting no-ops):

- **`relay_failure`** — kills a relay only once it's actually serving
  (state `RELAYING`) *and* overall connectivity has reached a real
  level, then measures time for connectivity to recover relative to
  current operational demand (see the recovery-metric writeup below
  for why it's demand-relative rather than a historical baseline).
- **`surveyor_failure`** — kills a UAV mid-transit to a PoI and measures
  how long until a *different* UAV picks up the orphaned task.
- **`double_failure`** — both of the above in one run, as a stress test.
- **`popup_priority`** — injects a priority-3 PoI mid-mission; confirms
  it gets picked up and reported.
- **`priority_ab_test`** — the more rigorous version of the above: runs
  the *same* seed and pop-up position/time twice (priority 3 vs.
  priority 1) under a constrained fleet, at a scouted moment where real
  contention exists (>=2 other pending PoIs), and compares pickup
  delay -- because score also weighs distance and relay cost, priority
  is a weighted factor, not an absolute override, and this test is
  what actually confirms that rather than assuming it.

Full results across 10 seeds (5 for `priority_ab_test`, given its
higher per-seed cost -- two full runs plus a scout each) are in the
table below.

**Full 10-seed results, after all fixes below (see next section):**

| Scenario | Result across 10 seeds |
|---|---|
| `relay_failure` | 100% completion, 0 collisions/geofence/charge violations in every seed. Recovery time (demand-relative, see below): mean 36.6s, range 1.0-169.0s |
| `surveyor_failure` | 100% completion, 0 collisions in every seed. Reassignment delay: exactly 5.0s every time (one replan cycle) |
| `double_failure` | 100% completion, 0 collisions/geofence/charge violations in every seed, even with two simultaneous failure types |
| `priority_ab_test` (5 seeds) | Priority measurably changed pickup order in 3/5 seeds under genuine contention (up to 1000s+ difference); made no difference in 2/5, because score also weighs distance and relay cost -- priority is a weighted factor, not an absolute override |

**Three real bugs were found and fixed while building and hardening
this, all worth knowing about if extending these scenarios further:**

1. **A UAV that died while holding `assigned_task` permanently
   stranded its PoI.** `UAV.kill()` only clears UAV-local state; nothing
   released the PoI back to `PENDING`, and `run_auction()` only ever
   looks at `PENDING` PoIs. This had existed since `failure_schedule`
   was first added to `Simulation.__init__` and had *never actually
   been exercised* by any test run up to this point -- every prior
   "verified across 10 seeds" claim in this README was about normal
   operation only. Fixed in `Simulation._apply_failures()`, mirroring
   the same release-on-departure pattern `_send_to_rth()` already used
   for deliberate recalls. Verified via a direct before/after
   comparison (see conversation history): same seed, same failure
   moment, orphaned PoI picked up by a different UAV, mission still
   reached 100%.
2. **The relay-failure recovery metric went through three revisions,
   each caught by checking actual output, not by assuming the logic
   was right:**
   - v1 guarded a connectivity-baseline capture with `n_connected_before
     is None`, but never actually assigned to that variable -- so
     whenever baseline connectivity happened to be captured as 0, the
     recovery condition (`n_conn >= min(baseline, ...)`) became
     trivially true on the very next step, reporting instant "recovery"
     that hadn't really happened.
   - v2 fixed that by reading from the logged step series post-hoc
     instead of fragile inline tracking, and improved the scouting
     logic (which had picked the very *first* relay to reach
     `RELAYING`, often before the rest of its own chain had arrived --
     baseline connectivity genuinely 0, nothing to recover from). But
     v2 still compared to a single historical pre-failure reading, and
     one seed produced a genuine (not spurious -- confirmed by
     averaging over a 20s window too) baseline of 28: the entire
     fleet momentarily mutually connected. "Recovery" to an
     unrepresentative historical peak isn't achievable or meaningful.
   - v3 (current) drops the historical-baseline idea entirely: recovery
     is now demand-relative -- is `n_connected` currently enough for
     *current* operational need (active surveyors + relays at that same
     instant)? This is the same definition `metrics.py`'s regular
     recovery-time calculation already used, so the two are now
     consistent with each other, and there's no baseline left to be an
     outlier. Full 10-seed result: every seed now produces a real
     recovery time (previously one seed produced `None`, "never
     recovered," which was really just an artifact of the broken
     baseline).
3. **A UAV returning to land could get stuck indefinitely a few metres
   from its assigned slot.** Found while investigating an unrelated
   anomaly: one relay-failure scenario logged 1002 charge violations
   in a single run, which is exactly the signature of the
   already-fixed "battery drains while idle" bug -- so it looked at
   first like that fix had regressed. Tracing the actual UAV showed
   something different and new: it sat at exactly 12.9m from its
   landing-pad slot, zero velocity, for over 500 simulated seconds,
   its battery pinned at 0 the whole time (RTH is a battery-draining
   state; only actually landing stops the drain). Root cause: ORCA is
   *purely reactive local collision avoidance with no path planning*
   (the paper says this explicitly) -- when many UAVs converge on the
   landing ring at once, a UAV whose direct line to its exact slot is
   blocked by an already-parked neighbour has no way to route *around*
   the obstacle, and can stall at the boundary indefinitely. This
   hadn't shown up in any prior baseline run (re-verified: still 0
   charge violations across all 10 baseline seeds after the fix,
   confirming it's real but rare, and specifically more likely to
   surface under the added disruption of a failure event). Fixed two
   ways: (a) loosened the RTH arrival threshold from 4m to 15m --
   `landing_pad_pos()` already guarantees >=38m between adjacent slots,
   so up to ~19m is provably safe from a separation-violation
   standpoint, and 15m leaves margin below that; (b) added a 90s
   stuck-timeout fallback that force-lands a UAV wherever it currently
   is if it's still within 60m of its slot and hasn't arrived by then
   -- defense in depth, not the primary fix. Verified: the exact
   scenario that produced 1002 violations now produces 0.

## Suggested next steps

1. Re-sweep `N_UAVS` (currently 28). That number was chosen *before*
   several later fixes (trunk-first slot filling, final-destination
   relay targeting, surveyor pacing) -- the point of diminishing
   returns may have shifted since.
2. Investigate the two seeds (7 and 10, in the earlier 10-seed latency
   sweep) where surveyor pacing made latency slightly *worse* instead
   of better -- never root-caused.
3. Run `scenarios.py` across more seeds (5 is a start, not a full
   statistical sample) for the report's actual results tables --
   especially `relay_failure` and `priority_ab_test`, which are the
   most novel findings so far.
4. "Network reconfiguration efficiency" (a rubric metric) isn't
   computed anywhere yet -- needs a definition before it can be
   reported.
5. "Recovery time" in `metrics.py`'s regular summary is still computed
   from ordinary relay churn, not tied to deliberate failures --
   `scenarios.py`'s failure-specific recovery metric is more honest
   for the rubric's actual question and should probably replace or
   supplement it in the final report numbers.
6. If pursuing tighter 10s compliance further: anticipatory relay
   pre-positioning ahead of task assignment, rather than only reactive
   dispatch at assignment time.
7. The interactive visualization currently ships one fixed scenario
   (seed 1) baked in at build time. Wiring up a seed/scenario selector
   would make it more useful for comparing seeds, or for showing one
   of the new failure/priority scenarios instead of the baseline run.
