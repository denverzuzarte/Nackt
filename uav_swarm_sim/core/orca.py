"""
ORCA: Optimal Reciprocal Collision Avoidance (van den Berg, Guy, Lin &
Manocha, 2011 -- "Reciprocal n-Body Collision Avoidance").

Implements:
  - the pairwise ORCA half-plane construction (paper Sec. 4.2, Eq. 5-6),
    using v_opt = current velocity for both agents (their recommended
    choice, Sec. 5.2 -- adapts to local density without needing the
    other agent's private preferred velocity);
  - the 2-D linear program that finds, within the max-speed disc, the
    velocity closest to the preferred velocity that satisfies every
    pairwise half-plane simultaneously (paper Sec. 5.1, linear_program_1
    and linear_program_2 in the reference RVO2 implementation of this
    paper's algorithm).

Simplification vs. the paper (stated explicitly in the report): when the
2-D LP is infeasible (very densely packed agents, Sec. 5.3), the paper
falls back to a 3-D linear program that finds the least-violating
velocity. We instead fall back to clamping onto the single most-violated
half-plane. This is adequate for our fleet density (<= ~25 UAVs over a
1000 m x 1000 m area with a 20 m separation requirement -- nowhere near
the "densely packed" regime the 3-D LP targets) and is flagged as a
Stage-2 fidelity improvement if higher densities are tested later.
"""
from __future__ import annotations
import numpy as np
from . import config as cfg

EPS = 1e-8


def _perp(v: np.ndarray) -> np.ndarray:
    """90-degree counter-clockwise rotation."""
    return np.array([-v[1], v[0]])


def _cross(a: np.ndarray, b: np.ndarray) -> float:
    return a[0] * b[1] - a[1] * b[0]


class Line:
    __slots__ = ("point", "direction")

    def __init__(self, point: np.ndarray, direction: np.ndarray):
        self.point = point
        self.direction = direction


def compute_orca_line(pos_a, vel_a, radius_a, pos_b, vel_b, radius_b,
                       tau: float, dt: float) -> Line:
    """
    The ORCA half-plane for agent A induced by agent B (paper Eq. 5-6),
    with v_opt_A = vel_a, v_opt_B = vel_b (current velocities).
    Geometry follows the standard construction: truncated-cone velocity
    obstacle, u = vector from relative velocity to the closest point on
    the VO boundary, half-plane starts at v_opt_A + u/2 (agents share
    responsibility equally).
    """
    rel_pos = pos_b - pos_a
    rel_vel = vel_a - vel_b
    dist_sq = float(rel_pos.dot(rel_pos))
    combined_r = radius_a + radius_b
    combined_r_sq = combined_r * combined_r

    if dist_sq > combined_r_sq:
        # not currently overlapping: cone truncated at 1/tau
        w = rel_vel - rel_pos / tau
        w_len_sq = float(w.dot(w))
        dot1 = float(w.dot(rel_pos))

        if dot1 < 0 and dot1 * dot1 > combined_r_sq * w_len_sq:
            # closest point is on the truncation circle
            w_len = np.sqrt(w_len_sq) if w_len_sq > EPS else EPS
            unit_w = w / w_len
            direction = np.array([unit_w[1], -unit_w[0]])
            u = (combined_r / tau - w_len) * unit_w
        else:
            # closest point is on one of the cone's legs
            leg = np.sqrt(max(dist_sq - combined_r_sq, 0.0))
            if _cross(rel_pos, w) > 0:
                direction = np.array([
                    rel_pos[0] * leg - rel_pos[1] * combined_r,
                    rel_pos[0] * combined_r + rel_pos[1] * leg
                ]) / max(dist_sq, EPS)
            else:
                direction = -np.array([
                    rel_pos[0] * leg + rel_pos[1] * combined_r,
                    -rel_pos[0] * combined_r + rel_pos[1] * leg
                ]) / max(dist_sq, EPS)
            dot2 = float(rel_vel.dot(direction))
            u = dot2 * direction - rel_vel
    else:
        # already overlapping (shouldn't normally happen given the
        # separation requirement, but handle it safely): push apart
        # using dt instead of tau, as in the reference implementation
        inv_dt = 1.0 / max(dt, EPS)
        w = rel_vel - rel_pos * inv_dt
        w_len = np.linalg.norm(w)
        w_len = w_len if w_len > EPS else EPS
        unit_w = w / w_len
        direction = np.array([unit_w[1], -unit_w[0]])
        u = (combined_r * inv_dt - w_len) * unit_w

    point = vel_a + 0.5 * u
    return Line(point=point, direction=direction)


# ---------------------------------------------------------------------
# 2-D linear programming (intersection of half-planes closest to a
# preferred point, within a max-speed disc)
# ---------------------------------------------------------------------
def _linear_program_1(lines, line_no, radius, opt_velocity):
    """Optimal point on lines[line_no], subject to lines[0..line_no-1]
    and the max-speed disc, closest to opt_velocity. Returns (ok, point)."""
    line = lines[line_no]
    dot_product = float(line.point.dot(line.direction))
    discriminant = dot_product ** 2 + radius ** 2 - float(line.point.dot(line.point))
    if discriminant < 0:
        return False, None
    sqrt_disc = np.sqrt(discriminant)
    t_left = -dot_product - sqrt_disc
    t_right = -dot_product + sqrt_disc

    for i in range(line_no):
        denom = _cross(line.direction, lines[i].direction)
        numer = _cross(lines[i].direction, line.point - lines[i].point)
        if abs(denom) <= EPS:
            if numer < 0:
                return False, None
            continue
        t = numer / denom
        if denom >= 0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return False, None

    t = float(line.direction.dot(opt_velocity - line.point))
    if t < t_left:
        t = t_left
    elif t > t_right:
        t = t_right
    return True, line.point + t * line.direction


def _linear_program_2(lines, radius, opt_velocity):
    """Returns (result_velocity, fail_index_or_None)."""
    if float(opt_velocity.dot(opt_velocity)) > radius ** 2:
        norm = np.linalg.norm(opt_velocity)
        result = (opt_velocity / norm) * radius if norm > EPS else np.zeros(2)
    else:
        result = opt_velocity.copy()

    for i, line in enumerate(lines):
        if _cross(line.direction, line.point - result) > 0:
            ok, new_result = _linear_program_1(lines, i, radius, opt_velocity)
            if not ok:
                return result, i
            result = new_result
    return result, None


def _clamp_to_line(result, line):
    """Fallback used in place of the paper's 3-D LP: project the current
    (infeasible) result onto the single most-violated half-plane's
    boundary. Simplification -- see module docstring."""
    d = float(line.direction.dot(result - line.point))
    return line.point + d * line.direction


def solve_velocity(pref_velocity, lines, max_speed):
    result, fail_idx = _linear_program_2(lines, max_speed, pref_velocity)
    if fail_idx is not None:
        result = _clamp_to_line(result, lines[fail_idx])
        # re-clip to max-speed disc after the fallback projection
        norm = np.linalg.norm(result)
        if norm > max_speed and norm > EPS:
            result = result / norm * max_speed
    return result


# ---------------------------------------------------------------------
def compute_new_velocities(agents, dt: float,
                            tau: float = None, neighbor_cutoff: float = None):
    """
    agents: list of objects with .pos, .vel, .id, and a preferred_velocity()
    method (UAV instances). Only "alive" agents participate.
    Returns dict {agent.id: new_velocity (np.ndarray)}.
    Synchronous update: every new velocity is computed from the *current*
    positions/velocities of all agents before any agent actually moves.
    """
    tau = tau or cfg.ORCA_TIME_HORIZON
    max_speed = cfg.UAV_SPEED
    radius = cfg.UAV_RADIUS
    if neighbor_cutoff is None:
        neighbor_cutoff = 2 * max_speed * tau + cfg.UAV_MIN_SEPARATION

    alive = [a for a in agents if a.alive]
    new_vel = {}
    for a in alive:
        pref_v = a.preferred_velocity()
        lines = []
        for b in alive:
            if b.id == a.id:
                continue
            if a.distance_to(b.pos) > neighbor_cutoff:
                continue
            lines.append(compute_orca_line(a.pos, a.vel, radius,
                                            b.pos, b.vel, radius, tau, dt))
        if lines:
            v_new = solve_velocity(pref_v, lines, max_speed)
        else:
            v_new = pref_v
        new_vel[a.id] = v_new
    return new_vel
