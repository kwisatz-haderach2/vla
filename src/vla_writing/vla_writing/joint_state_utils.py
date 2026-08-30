"""Utilities for keeping finite-joint trajectories on a canonical branch.

The UR5e description uses *revolute* joints with finite limits.  Gazebo and a
``JointTrajectoryController`` can nevertheless report an equivalent angle
after a trajectory crosses a ``2*pi`` branch (for example ``12.4`` radians is
the same pose as ``0.0`` radians).  MoveIt quite correctly rejects that value
when it is used as the next planning start state.  This module contains no
ROS imports, so it can be used by the parser tests as well as by the ROS1
executor.

The important distinction is between a physical joint limit and an angle's
period.  We never change the URDF joint type or silently turn a revolute joint
into a continuous one.  Instead, an equivalent value ``angle + k*2*pi`` is
selected inside the configured finite interval and, when possible, closest to
the preceding value.  That gives controllers a short, deterministic path
without changing the robot model semantics.
"""

from __future__ import annotations

import copy
import math
from typing import Any, Dict, Iterable, List, Mapping, MutableSequence, Optional, Sequence, Tuple


TWO_PI = 2.0 * math.pi

# The official UR5e limits are +/-360 degrees for all joints except the elbow,
# which MoveIt conventionally constrains to +/-180 degrees.  The writing
# scene may supply narrower limits (for example +/-180 degrees for the pan and
# wrist-3 joints); callers can pass those in ``limits`` and they take priority.
UR5E_JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "shoulder_pan_joint": (-TWO_PI, TWO_PI),
    "shoulder_lift_joint": (-TWO_PI, TWO_PI),
    "elbow_joint": (-math.pi, math.pi),
    "wrist_1_joint": (-TWO_PI, TWO_PI),
    "wrist_2_joint": (-TWO_PI, TWO_PI),
    "wrist_3_joint": (-TWO_PI, TWO_PI),
}


def wrap_to_pi(angle: float) -> float:
    """Return the equivalent angle in ``[-pi, pi)``.

    ``math.fmod`` is intentionally avoided here: Python's modulo operation
    gives the desired result for negative angles and remains stable for the
    values normally encountered in a robot trajectory.  A non-finite value is
    a programming/sensor error and is rejected explicitly instead of being
    propagated into a ROS message.
    """

    value = float(angle)
    if not math.isfinite(value):
        raise ValueError("joint angle must be finite, got {!r}".format(angle))
    return (value + math.pi) % TWO_PI - math.pi


def _limits_mapping(limits: Optional[Mapping[str, Any]]) -> Mapping[str, Any]:
    """Accept either a direct map or a MoveIt-style ``joint_limits`` map."""

    if not isinstance(limits, Mapping):
        return UR5E_JOINT_LIMITS
    nested = limits.get("joint_limits")
    if isinstance(nested, Mapping):
        return nested
    return limits


def _lookup_limit(name: str, limits: Optional[Mapping[str, Any]]) -> Optional[Tuple[float, float]]:
    """Resolve a joint limit from several common YAML naming conventions."""

    table = _limits_mapping(limits)
    candidates = [str(name)]
    # The upstream UR description uses keys such as ``shoulder_pan`` while
    # MoveIt state/trajectory messages use ``shoulder_pan_joint``.
    if str(name).endswith("_joint"):
        candidates.append(str(name)[:-6])
    else:
        candidates.append(str(name) + "_joint")
    for candidate in candidates:
        if candidate not in table:
            continue
        value = table[candidate]
        if isinstance(value, Mapping):
            lower = value.get("min_position", value.get("lower"))
            upper = value.get("max_position", value.get("upper"))
            if lower is None or upper is None:
                continue
        elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) >= 2:
            lower, upper = value[0], value[1]
        else:
            continue
        try:
            lower_f, upper_f = float(lower), float(upper)
        except (TypeError, ValueError):
            continue
        if math.isfinite(lower_f) and math.isfinite(upper_f) and lower_f <= upper_f:
            return lower_f, upper_f
    # If the caller supplied a partial map, retain the safe official default
    # for joints not mentioned there.
    default = UR5E_JOINT_LIMITS.get(str(name))
    return default


def joint_limits(
    name: str,
    limits: Optional[Mapping[str, Any]] = None,
) -> Optional[Tuple[float, float]]:
    """Public limit lookup helper used by the executor and tests."""

    return _lookup_limit(name, limits)


def _candidate_values(
    angle: float,
    lower: float,
    upper: float,
    period: float,
    margin: float,
) -> List[float]:
    """Enumerate equivalent values that fit inside a finite interval."""

    if not math.isfinite(period) or period <= 0.0:
        raise ValueError("period must be a positive finite number")
    lo = float(lower) + max(0.0, float(margin))
    hi = float(upper) - max(0.0, float(margin))
    if lo > hi:
        # A margin larger than half the interval should not make the helper
        # unusable.  Falling back to the midpoint still returns a bounded
        # command and is safer than emitting an invalid trajectory.
        midpoint = 0.5 * (float(lower) + float(upper))
        lo = hi = midpoint

    # Include one extra integer on either side to avoid rounding gaps at a
    # boundary.  The intervals involved are tiny (at most a few dozen turns),
    # so an explicit list is clearer and safer than clever floating-point code.
    first = int(math.ceil((lo - angle) / period - 1e-12))
    last = int(math.floor((hi - angle) / period + 1e-12))
    return [angle + k * period for k in range(first, last + 1)]


def canonical_angle(
    angle: float,
    lower: float,
    upper: float,
    reference: Optional[float] = None,
    period: float = TWO_PI,
    margin: float = 1e-6,
) -> float:
    """Choose a bounded equivalent representation of ``angle``.

    If ``reference`` is supplied, the candidate with the smallest absolute
    difference from it is returned.  This is the key operation for successive
    trajectory points: a planner may return ``-6.2`` after a previous point at
    ``0.1``, but the equivalent ``0.08`` is the short and controller-friendly
    continuation.  Without a reference, the candidate closest to zero is
    selected, making a deterministic canonical state for recovery.
    """

    value = float(angle)
    lo = float(lower)
    hi = float(upper)
    if not math.isfinite(value):
        raise ValueError("joint angle must be finite, got {!r}".format(angle))
    if not math.isfinite(lo) or not math.isfinite(hi) or lo > hi:
        raise ValueError("invalid joint limits [{!r}, {!r}]".format(lower, upper))
    if reference is not None:
        reference = float(reference)
        if not math.isfinite(reference):
            reference = None

    candidates = _candidate_values(value, lo, hi, float(period), float(margin))
    if not candidates:
        # A finite interval narrower than one period can theoretically contain
        # no equivalent class for a malformed input.  Clamp the nearest
        # wrapped value rather than allowing an out-of-bounds command through.
        wrapped = wrap_to_pi(value)
        return min(max(wrapped, lo), hi)
    if reference is None:
        return min(candidates, key=lambda candidate: (abs(candidate), candidate))
    return min(candidates, key=lambda candidate: (abs(candidate - reference), abs(candidate), candidate))


def wrap_angle_to_limits(
    angle: float,
    limits: Tuple[float, float],
    reference: Optional[float] = None,
    period: float = TWO_PI,
    margin: float = 1e-6,
) -> float:
    """Alias with a tuple-oriented signature for small standalone clients."""

    return canonical_angle(angle, limits[0], limits[1], reference, period, margin)


def normalize_joint_positions(
    joint_names: Sequence[str],
    positions: Sequence[float],
    limits: Optional[Mapping[str, Any]] = None,
    previous: Optional[Mapping[str, float]] = None,
    margin: float = 1e-6,
    period: float = TWO_PI,
) -> List[float]:
    """Normalize one joint-position vector while preserving unknown joints.

    Unknown joint names are copied unchanged because a caller may pass a
    trajectory containing fixed or mimic joints that are not part of the UR5e
    six-axis table.  A missing limit is therefore not treated as an error.
    """

    names = list(joint_names)
    values = list(positions)
    if len(names) != len(values):
        raise ValueError("joint_names and positions have different lengths")
    result: List[float] = []
    previous_map = previous if isinstance(previous, Mapping) else {}
    for name, value in zip(names, values):
        bound = _lookup_limit(str(name), limits)
        if bound is None:
            numeric = float(value)
            if not math.isfinite(numeric):
                raise ValueError("joint angle must be finite, got {!r}".format(value))
            result.append(numeric)
            continue
        reference = previous_map.get(str(name))
        result.append(canonical_angle(value, bound[0], bound[1], reference, period, margin))
    return result


def canonicalize_joint_state(
    joint_names: Sequence[str],
    positions: Sequence[float],
    limits: Optional[Mapping[str, Any]] = None,
    margin: float = 1e-6,
) -> Dict[str, float]:
    """Return a name→position map in a deterministic bounded branch."""

    values = normalize_joint_positions(joint_names, positions, limits=limits, margin=margin)
    return {str(name): float(value) for name, value in zip(joint_names, values)}


def invalid_joint_state(
    joint_names: Sequence[str],
    positions: Sequence[float],
    limits: Optional[Mapping[str, Any]] = None,
    tolerance: float = 0.1,
) -> Dict[str, float]:
    """Return joints that are materially outside their configured limits.

    The tolerance mirrors MoveIt's ``start_state_max_bounds_error`` notion:
    values a few floating-point ulps beyond an endpoint are harmless, while a
    multi-turn value such as ``12.4`` is reported for recovery.
    """

    names = list(joint_names)
    values = list(positions)
    if len(names) != len(values):
        raise ValueError("joint_names and positions have different lengths")
    result: Dict[str, float] = {}
    tol = max(0.0, float(tolerance))
    for name, value in zip(names, values):
        numeric = float(value)
        bound = _lookup_limit(str(name), limits)
        if not math.isfinite(numeric):
            result[str(name)] = numeric
        elif bound is not None and (numeric < bound[0] - tol or numeric > bound[1] + tol):
            result[str(name)] = numeric
    return result


def _trajectory_container(trajectory: Any) -> Any:
    """Return a JointTrajectory-like object from RobotTrajectory or itself."""

    if trajectory is None:
        return None
    joint_trajectory = getattr(trajectory, "joint_trajectory", None)
    if joint_trajectory is not None and hasattr(joint_trajectory, "joint_names"):
        return joint_trajectory
    if hasattr(trajectory, "joint_names") and hasattr(trajectory, "points"):
        return trajectory
    return None


def normalize_trajectory(
    trajectory: Any,
    limits: Optional[Mapping[str, Any]] = None,
    reference: Optional[Mapping[str, float]] = None,
    margin: float = 1e-6,
    period: float = TWO_PI,
) -> Any:
    """Copy and normalize a ROS ``RobotTrajectory``/``JointTrajectory``.

    Only the position field is changed.  Velocities and accelerations are
    physically invariant under adding an integer number of turns, and keeping
    them intact avoids invalidating a controller's timing/retiming metadata.
    The input object is deep-copied whenever possible so callers can retain
    MoveIt's original result for diagnostics.
    """

    container = _trajectory_container(trajectory)
    if container is None:
        return trajectory
    try:
        output = copy.deepcopy(trajectory)
    except Exception:
        # ROS messages are normally deepcopy-able.  If a vendor message class
        # is not, mutate it in place as a best-effort fallback.
        output = trajectory
    target = _trajectory_container(output)
    if target is None:
        return output
    names = [str(name) for name in getattr(target, "joint_names", [])]
    previous: Dict[str, float] = {}
    if isinstance(reference, Mapping):
        for name, value in reference.items():
            try:
                previous[str(name)] = float(value)
            except (TypeError, ValueError):
                continue

    for point in list(getattr(target, "points", []) or []):
        positions = list(getattr(point, "positions", []) or [])
        if not positions:
            continue
        normalized = normalize_joint_positions(
            names,
            positions,
            limits=limits,
            previous=previous,
            margin=margin,
            period=period,
        )
        try:
            point.positions = normalized
        except Exception:
            try:
                point.positions[:] = normalized
            except Exception:
                # A malformed third-party message should not crash a dry-run
                # or diagnostic caller; leave that point untouched.
                continue
        previous = {name: value for name, value in zip(names, normalized)}
    return output


# Spellings/aliases kept intentionally small and explicit for downstream code
# written by the ROS2 force-control teammate.
normalize_robot_trajectory = normalize_trajectory
normalize_joint_trajectory = normalize_trajectory
shortest_equivalent = canonical_angle


__all__ = [
    "TWO_PI",
    "UR5E_JOINT_LIMITS",
    "wrap_to_pi",
    "joint_limits",
    "canonical_angle",
    "wrap_angle_to_limits",
    "normalize_joint_positions",
    "canonicalize_joint_state",
    "invalid_joint_state",
    "normalize_trajectory",
    "normalize_robot_trajectory",
    "normalize_joint_trajectory",
    "shortest_equivalent",
]
