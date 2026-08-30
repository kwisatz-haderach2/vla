"""Deterministic linear interpolation helpers for writing trajectories."""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple, Union

from .data_types import Point2D, Stroke, coerce_point


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a.u - b.u, a.v - b.v)


def interpolate_polyline(
    points: Iterable[Union[Point2D, Sequence[float]]],
    spacing: float = 0.002,
    include_start: bool = True,
    include_end: bool = True,
) -> List[Point2D]:
    """Densely sample a 2-D polyline at approximately ``spacing`` metres.

    For each source segment ``P0→P1`` this uses ``ceil(length / spacing)``
    equal intervals, exactly as specified in the project plan.  Original
    vertices are retained, and duplicate points at segment joins are removed.
    """

    source = [coerce_point(p) for p in points]
    if not source:
        return []
    if len(source) == 1 or spacing is None or float(spacing) <= 0:
        result = [p.copy() for p in source]
        if not include_start and result:
            result = result[1:]
        if not include_end and result:
            result = result[:-1]
        return result
    step = float(spacing)
    result: List[Point2D] = [source[0].copy()]
    for start, end in zip(source[:-1], source[1:]):
        dx = end.u - start.u
        dy = end.v - start.v
        distance = math.hypot(dx, dy)
        if distance <= 1e-12:
            continue
        intervals = max(1, int(math.ceil(distance / step)))
        for index in range(1, intervals + 1):
            fraction = float(index) / float(intervals)
            point = Point2D(start.u + fraction * dx, start.v + fraction * dy)
            if _distance(result[-1], point) > 1e-12:
                result.append(point)
    if not include_start and result:
        result = result[1:]
    if not include_end and result:
        result = result[:-1]
    return result


def interpolate_stroke(stroke: Stroke, spacing: float = 0.002, close: Optional[bool] = None) -> Stroke:
    """Return an interpolated copy of one stroke."""

    should_close = stroke.closed if close is None else bool(close)
    points = [p.copy() for p in stroke.points]
    if should_close and len(points) >= 3 and _distance(points[0], points[-1]) > 1e-12:
        points.append(points[0].copy())
    sampled = interpolate_polyline(points, spacing)
    return Stroke(sampled, should_close, dict(stroke.metadata))


def interpolate_strokes(strokes: Iterable[Stroke], spacing: float = 0.002) -> List[Stroke]:
    return [interpolate_stroke(stroke, spacing) for stroke in strokes if len(stroke.points) >= 1]


def resample_points(points: Iterable[Union[Point2D, Sequence[float]]], spacing: float = 0.002) -> List[Point2D]:
    """Alias retained for image/parser code and external scripts."""

    return interpolate_polyline(points, spacing)


def polyline_length(points: Iterable[Union[Point2D, Sequence[float]]], closed: bool = False) -> float:
    source = [coerce_point(p) for p in points]
    if len(source) < 2:
        return 0.0
    total = sum(_distance(a, b) for a, b in zip(source[:-1], source[1:]))
    if closed:
        total += _distance(source[-1], source[0])
    return total


__all__ = [
    "interpolate_polyline",
    "interpolate_stroke",
    "interpolate_strokes",
    "resample_points",
    "polyline_length",
]
