"""
Core entities: UAV and PoI.

Design note: this stays deliberately simple (kinematic point-mass UAVs,
no attitude/dynamics) so the whole allocation + connectivity + failure +
battery loop can be built and tested within the time budget. This is
stated explicitly as a simplification in the report (see README).
"""
from __future__ import annotations
import numpy as np
from enum import Enum, auto
from . import config as cfg


class UAVState(Enum):
    IDLE = auto()          # free, waiting near GCS
    TRANSIT = auto()       # travelling to a PoI or relay point
    SURVEYING = auto()     # dwelling at a PoI, collecting data
    AWAITING_REPORT = auto()  # data collected, holding position until a
                               # live path to GCS delivers it (COMPETITION
                               # SPEC: max 10s detection-to-report) -- the
                               # UAV stays an "owner" the relay backbone
                               # tries to reach even after collection ends
    RELAYING = auto()      # holding position as a communication relay
    RTH = auto()           # returning to home (GCS) to recharge
    CHARGING = auto()      # at GCS, recharging
    FAILED = auto()        # dead / non-functional


class UAVRole(Enum):
    NONE = auto()
    SURVEYOR = auto()
    RELAY = auto()


class UAV:
    def __init__(self, uid: int, pos: np.ndarray):
        self.id = uid
        self.pos = np.array(pos, dtype=float)
        self.vel = np.zeros(2, dtype=float)  # current velocity, used by ORCA
        self.target = None                 # np.ndarray or None
        self.speed_cap = cfg.UAV_SPEED     # backbone-gating throttle (see
                                            # sim.py _throttle_backbone_advance);
                                            # 0 means "hold, let the chain
                                            # behind you catch up"
        self.state = UAVState.IDLE
        self.role = UAVRole.NONE
        self.battery = cfg.BATTERY_CAPACITY
        self.assigned_task = None          # PoI id currently assigned (survey)
        self.assigned_relay_slot = None     # (chain_owner_id, slot_index) if relaying
        self.survey_timer = 0.0            # counts down while SURVEYING
        self.connected_to_gcs = False       # updated each step by connectivity module
        self.path_hops_to_gcs = None        # int or None
        self.alive = True
        self.hold_for_relay = False  # True: pause final approach and let
                                      # own relay chain catch up (see
                                      # sim.py _update_surveyor_pacing)
        self.hold_since = None       # sim time the current hold began,
                                      # or None if not currently holding
        self.rth_start_time = None   # sim time RTH began, for the
                                      # stuck-landing timeout fallback

        # bookkeeping for metrics
        self.time_connected = 0.0
        self.time_disconnected = 0.0
        self.total_time_active = 0.0

    # -----------------------------------------------------------------
    def distance_to(self, other_pos: np.ndarray) -> float:
        return float(np.linalg.norm(self.pos - other_pos))

    def set_target(self, target_pos: np.ndarray):
        self.target = np.array(target_pos, dtype=float)

    def preferred_velocity(self) -> np.ndarray:
        """Velocity the UAV would fly at with no other traffic around --
        straight toward its current target at cruise speed. This is the
        'v_pref' input to ORCA (van den Berg et al. 2011)."""
        if self.target is None or not self.alive:
            return np.zeros(2)
        if self.hold_for_relay:
            return np.zeros(2)
        direction = self.target - self.pos
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            return np.zeros(2)
        speed = min(self.speed_cap, dist / max(cfg.DT, 1e-6))
        return (direction / dist) * speed

    def apply_velocity(self, v: np.ndarray, dt: float):
        """Advance position using a velocity already resolved by the
        collision-avoidance layer (ORCA), and remember it as current
        velocity for next step's ORCA computation."""
        if not self.alive:
            return
        self.vel = np.array(v, dtype=float)
        self.pos = self.pos + self.vel * dt

    # States in which the UAV is actually airborne and burning its 20-min
    # flight-time budget. IDLE means "docked at GCS, not yet launched (or
    # briefly hovering there awaiting reassignment)" -- Stage-1
    # simplification: we don't model hover power draw while idle, only
    # active-flight drain, since IDLE UAVs are either literally on the
    # ground at the GCS or only briefly airborne between the 5 s replan
    # ticks. Noted explicitly in the report.
    FLYING_STATES = (UAVState.TRANSIT, UAVState.SURVEYING,
                      UAVState.RELAYING, UAVState.RTH)

    def drain_battery(self, dt: float):
        if not self.alive or self.state not in self.FLYING_STATES:
            return
        self.battery = max(0.0, self.battery - cfg.BATTERY_DRAIN_RATE * dt)

    def recharge(self, dt: float):
        self.battery = min(cfg.BATTERY_CAPACITY,
                            self.battery + dt / cfg.BATTERY_RECHARGE_TIME)

    def kill(self):
        self.alive = False
        self.state = UAVState.FAILED
        self.role = UAVRole.NONE
        self.target = None
        self.assigned_relay_slot = None
        # NOTE: does NOT clear assigned_task -- that's deliberately left to
        # the caller (Simulation._apply_failures), which needs to decide
        # what happens to the PoI (release to PENDING, or leave alone if
        # already SURVEYED) before this UAV stops being addressable by id.
        # A UAV class method shouldn't reach into the PoI list itself.

    def __repr__(self):
        return (f"UAV({self.id}, state={self.state.name}, role={self.role.name}, "
                f"batt={self.battery:.2f}, pos=({self.pos[0]:.0f},{self.pos[1]:.0f}))")


class PoIStatus(Enum):
    PENDING = auto()
    ASSIGNED = auto()
    SURVEYED = auto()


class PoI:
    def __init__(self, pid: int, pos: np.ndarray, priority: int = 1,
                 reveal_time: float = 0.0):
        self.id = pid
        self.pos = np.array(pos, dtype=float)
        self.priority = priority           # 1 = normal, 2 = high, 3 = critical
        self.reveal_time = reveal_time     # sim time at which this PoI becomes known
        self.status = PoIStatus.PENDING
        self.assigned_uav = None
        self.survey_start_time = None
        self.survey_complete_time = None
        self.was_connected_during_survey = None  # bool, set when surveyed

        # COMPETITION SPEC: max 10 s between detection and reporting to GCS.
        # detection_time = moment a UAV arrives at the PoI (data collected).
        # report_time = first moment thereafter the detecting UAV (or its
        # relay chain) has a live multi-hop path to the GCS.
        self.detection_time = None
        self.detecting_uav_id = None
        self.report_time = None

    @property
    def report_latency(self):
        if self.detection_time is None or self.report_time is None:
            return None
        return self.report_time - self.detection_time

    def is_revealed(self, t: float) -> bool:
        return t >= self.reveal_time

    def __repr__(self):
        return (f"PoI({self.id}, prio={self.priority}, status={self.status.name}, "
                f"pos=({self.pos[0]:.0f},{self.pos[1]:.0f}))")
