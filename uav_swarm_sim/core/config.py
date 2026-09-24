"""
Central configuration for the UAV swarm disaster-response simulation.
Keep every tunable parameter here so the report's §9.1 (simulation setup)
can just point at this file.

Values below marked "COMPETITION SPEC" are taken directly from the
organizers' mission-constraints sheet and must not be changed without
updating the report. Everything else is an engineering choice we made
to satisfy those constraints and is open to tuning.
"""

# ---------------------------------------------------------------------------
# World / geofence -- COMPETITION SPEC
# ---------------------------------------------------------------------------
# Operational area: 1000 m x 1000 m square, origin at its bottom-left corner.
OP_AREA_MIN = (0.0, 0.0)
OP_AREA_MAX = (1000.0, 1000.0)

# Operational center (GCS): all UAVs take off from and must land here.
# Per the mission diagram it sits 75 m outside the operational area, to
# its west, roughly level with the area's vertical centre.
GCS_POS = (-75.0, 500.0)

# Geofence: UAVs must not leave this box. We use a permissive margin
# around {GCS + operational area} rather than a tight polygon around just
# the launch corridor -- stated simplification, easy to tighten later.
# The west margin in particular needs real room: near the 45-min mission
# deadline, every UAV still airborne is forced to RTH at once (regardless
# of individual battery level), so the GCS briefly has to accommodate the
# whole fleet parked/charging simultaneously -- a real staging area would
# be sized for this "rush hour," so we do the same here rather than
# treating the resulting mild boundary pressure as a control failure.
GEOFENCE_MIN = (-400.0, -150.0)
GEOFENCE_MAX = (1050.0, 1150.0)

MAX_ALTITUDE = 100.0          # m -- COMPETITION SPEC. Not modelled (2-D sim,
                               # single flight layer assumed); noted as a
                               # Stage-1 simplification in the report.

DT = 0.5                       # simulation timestep, seconds. Tried 0.25 s
                               # to improve ORCA reaction time and
                               # reporting-latency precision; combined with
                               # a larger UAV_RADIUS it instead made
                               # collisions much worse (ORCA's "already
                               # overlapping" fallback branch scales with
                               # 1/dt, so shrinking dt while also growing
                               # the radius amplified its push-apart
                               # velocity into oscillatory overcorrection).
                               # Reverted to isolate variables -- see
                               # README for what's still worth trying.
MAX_SIM_TIME = 45 * 60.0      # COMPETITION SPEC: 45 min mission operation

# ---------------------------------------------------------------------------
# UAV kinematics -- COMPETITION SPEC
# ---------------------------------------------------------------------------
UAV_SPEED = 5.0                # m/s -- COMPETITION SPEC: max speed
UAV_MIN_SEPARATION = 20.0      # m -- COMPETITION SPEC: min distance between vehicles
# ORCA's "radius" is the avoidance radius each agent claims; two agents
# each claiming half of the required separation keeps them >= 20 m apart
# (paper's combined_radius = r_A + r_B). A small buffer is added since
# ORCA's tau-based guarantee is a *sufficient*-for-tau-seconds condition,
# not an exact minimum-distance controller -- the buffer absorbs
# discretisation error from our DT-sized steps.
UAV_RADIUS = UAV_MIN_SEPARATION / 2.0 + 1.5   # m -- extra buffer absorbs
                                                # some reaction lag on
                                                # abrupt target changes,
                                                # without pushing the
                                                # "already overlapping"
                                                # ORCA fallback branch into
                                                # triggering routinely
                                                # (see DT note above)
ORCA_TIME_HORIZON = 8.0         # s, ORCA look-ahead window (tau). Swept
                               # 3-10 s at fixed DT and radius: min
                               # observed separation improved from ~15 m
                               # (tau=4) to ~18 m (tau=8) before degrading
                               # again at tau=10, so 8 s is the sweet spot
                               # for this fleet size/density rather than
                               # "bigger is always safer."

# ---------------------------------------------------------------------------
# Communication model (distance-threshold, Ponda/Swarm-Relays style)
# ---------------------------------------------------------------------------
COMM_RANGE = 100.0              # m -- COMPETITION SPEC: max comm range
COMM_SAFE = 75.0                 # m, "safe zone" -- reliable, no action needed
COMM_CRITICAL = 90.0             # m, "critical zone" -- link degrading
# beyond COMM_CRITICAL up to COMM_RANGE: "break-away zone" -- link about to be lost
# beyond COMM_RANGE: no link

MAX_REPORT_LATENCY = 10.0       # s -- COMPETITION SPEC: max time between PoI
                                 # detection and reporting to the GCS

# ---------------------------------------------------------------------------
# Battery / endurance -- COMPETITION SPEC: 20 min max flight time
# ---------------------------------------------------------------------------
BATTERY_CAPACITY = 1.0        # normalised, 1.0 = full
BATTERY_DRAIN_RATE = 1.0 / (20 * 60.0)   # full battery lasts exactly 20 min of flight
BATTERY_RTH_THRESHOLD = 0.30  # return-to-home triggered at this fraction remaining
BATTERY_RECHARGE_TIME = 20 * 60.0  # s, time to fully recharge at GCS (not exercised
                                     # within a single 45-min mission at this rate,
                                     # kept for completeness / future multi-sortie work)
RTH_ARRIVAL_THRESHOLD = 15.0    # m -- how close to its landing-pad slot a
                                 # UAV must get before being considered
                                 # "landed" (CHARGING). Looser than the 4m
                                 # used for PoI/relay arrival on purpose:
                                 # ORCA is purely reactive local collision
                                 # avoidance with no path planning (the
                                 # paper itself says so explicitly), so
                                 # when many UAVs converge on the landing
                                 # ring at once, a UAV whose direct line to
                                 # its exact slot is blocked by an
                                 # already-parked neighbour can stall
                                 # indefinitely a few metres short --
                                 # observed directly: one UAV sat at 12.9m
                                 # from its slot, zero velocity, for over
                                 # 500s before finally breaking free.
                                 # landing_pad_pos() guarantees >=38m
                                 # between adjacent slots, so up to ~19m is
                                 # provably safe (no two "arrived" UAVs on
                                 # adjacent slots could be under the 20m
                                 # separation requirement); 15m leaves
                                 # margin below that bound.
RTH_STUCK_TIMEOUT = 90.0        # s -- if a UAV has been in RTH this long
                                 # without reaching even the loosened
                                 # threshold, force it to land wherever it
                                 # currently is (see RTH_STUCK_RADIUS).
                                 # Defense in depth, not the primary fix:
                                 # the loosened threshold above should
                                 # resolve most cases outright.
RTH_STUCK_RADIUS = 60.0         # m -- the force-land fallback only fires
                                 # if the UAV is already reasonably close
                                 # to its slot (genuinely stuck nearby),
                                 # not for a UAV that's still legitimately
                                 # far away and simply hasn't arrived yet.

RTH_TIME_SAFETY_MARGIN = 90.0  # s, extra buffer when checking "can I still get
                                 # home before the 45-min mission deadline".
                                 # Needs to be generous, not tight: UAVs whose
                                 # own straight-line distance-to-home is small
                                 # don't trigger this check until very late in
                                 # the mission (their naive time-to-home is
                                 # small), which means many of them end up
                                 # converging on the single GCS point in the
                                 # same final window. ORCA-mediated mutual
                                 # avoidance during that rush measurably slows
                                 # them down relative to the unobstructed
                                 # straight-line estimate this check uses, so
                                 # a thin margin (originally 20s) gets eaten
                                 # by congestion and some UAVs miss the
                                 # deadline. See README for the measurement.

# ---------------------------------------------------------------------------
# Task allocation (CBAA-lite)
# ---------------------------------------------------------------------------
TASK_SERVICE_TIME = 15.0      # s, time a UAV must dwell at a PoI to "survey" it
SURVEYOR_HOLD_MAX_S = 60.0    # s -- cap on how long a surveyor will hold its
                               # final approach waiting for its relay chain
                               # (see sim.py _update_surveyor_pacing). A held
                               # UAV's TRANSIT state still drains battery
                               # (holding is "hovering", not "landed"), so an
                               # unbounded hold could burn enough flight time
                               # to force it into RTH before ever completing
                               # the survey -- observed as a real regression
                               # (one seed's completion rate dropped from
                               # 100% to 80%) before this cap was added.
                               # Past this cap it proceeds anyway, accepting
                               # a latency hit rather than risking the task
                               # never completing at all.
REPLAN_INTERVAL = 5.0         # s, how often the allocation is re-run
PRIORITY_WEIGHTS = {1: 1.0, 2: 2.0, 3: 4.0}  # PoI priority level -> score multiplier

# ---------------------------------------------------------------------------
# Failure injection (for robustness testing, §10)
# ---------------------------------------------------------------------------
FAILURE_SCHEDULE = []         # list of (time, uav_id) tuples; filled per-scenario

# ---------------------------------------------------------------------------
# Fleet composition (default scenario)
# ---------------------------------------------------------------------------
N_UAVS = 28                    # NOT specified by the competition sheet -- our
                                # choice. Swept 20/24/28/32 across 5 seeds:
                                # collisions stayed at 0 throughout (an ORCA/
                                # relay-planning property, not fleet-size
                                # dependent), but relay-shortage task drops and
                                # report latency both improved sharply up to
                                # ~28 and only marginally beyond it. 28 is
                                # chosen as the point of diminishing returns,
                                # not just "more is better". See README.
N_POIS = 10                    # COMPETITION SPEC: 10 PoIs
POI_REVEAL_WINDOW = (0.0, 25 * 60.0)  # PoIs spawn at random times in this window
                                        # (COMPETITION SPEC says "spawned randomly
                                        # in position and time"; window chosen so
                                        # there's realistically time to survey them
                                        # all before the 45-min deadline)
RANDOM_SEED = 42
