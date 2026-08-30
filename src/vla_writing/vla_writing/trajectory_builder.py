"""Convert laid-out paper strokes into Cartesian writing waypoints.

The builder is deliberately independent of MoveIt.  It returns small tuples
``(u, v, z, quaternion)`` in the paper frame; :class:`MoveItExecutor` turns
those tuples into ``geometry_msgs/Pose`` objects after applying the TF
transform to the robot planning frame.  This keeps English, Hanzi and image
inputs on exactly the same motion path.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

from .data_types import Point2D, Stroke, coerce_point
from .interpolator import interpolate_polyline


Quaternion = Tuple[float, float, float, float]


@dataclass
class PaperWaypoint:
    """One Cartesian point in ``paper_frame`` coordinates."""

    u: float
    v: float
    z: float
    orientation: Quaternion = (0.0, 0.0, 0.0, 1.0)
    pen_down: bool = False

    def as_tuple(self) -> Tuple[float, float, float, Quaternion, bool]:
        return (self.u, self.v, self.z, self.orientation, self.pen_down)


class TrajectoryBuilder:
    """Build lift/write/lift trajectories from 2-D strokes."""

    def __init__(
        self,
        write_z: float = 0.0015,
        lift_z: float = 0.030,
        point_spacing: float = 0.002,
        orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
        paper_frame: str = "paper_frame",
        base_frame: str = "base_link",
    ):
        self.write_z = float(write_z)
        self.lift_z = float(lift_z)
        self.point_spacing = float(point_spacing)
        values = list(orientation)
        if len(values) != 4:
            values = [0.0, 0.0, 0.0, 1.0]
        self.orientation: Quaternion = tuple(float(v) for v in values)  # type: ignore[assignment]
        self.paper_frame = paper_frame
        self.base_frame = base_frame

    @classmethod
    def from_config(cls, config: Mapping[str, Any], **kwargs: Any) -> "TrajectoryBuilder":
        """Construct a builder from the nested ``writing.yaml`` mapping."""

        paper = config.get("paper", {}) if isinstance(config, Mapping) else {}
        motion = config.get("motion", {}) if isinstance(config, Mapping) else {}
        moveit = config.get("moveit", {}) if isinstance(config, Mapping) else {}
        return cls(
            write_z=motion.get("write_z", 0.0015),
            lift_z=motion.get("lift_z", 0.030),
            point_spacing=motion.get("point_spacing", 0.002),
            orientation=motion.get("orientation", (0.0, 0.0, 0.0, 1.0)),
            paper_frame=paper.get("frame", "paper_frame") if isinstance(paper, Mapping) else "paper_frame",
            base_frame=moveit.get("base_frame", "base_link") if isinstance(moveit, Mapping) else "base_link",
            **kwargs
        )

    def _pose(self, point: Point2D, z: float, pen_down: bool) -> PaperWaypoint:
        return PaperWaypoint(point.u, point.v, float(z), self.orientation, pen_down)

    def paper_to_base_pose(self, point: Union[Point2D, Sequence[float]], z: Optional[float] = None) -> PaperWaypoint:
        """Return a logical paper-frame waypoint for ``(u,v)``.

        The name is retained for compatibility with the design document.  TF
        conversion is intentionally left to ``MoveItExecutor``; this method
        therefore returns a :class:`PaperWaypoint` whose ``v`` still follows
        the paper's downwards logical convention.
        """

        p = coerce_point(point)
        return self._pose(p, self.write_z if z is None else float(z), z is None or abs(float(z) - self.write_z) < 1e-12)

    def stroke_to_waypoints(
        self,
        stroke: Stroke,
        orientation: Optional[Sequence[float]] = None,
    ) -> List[PaperWaypoint]:
        """Return one stroke's complete lift/descend/write/lift sequence."""

        if stroke is None or len(stroke.points) < 2:
            return []
        old_orientation = self.orientation
        if orientation is not None and len(orientation) == 4:
            self.orientation = tuple(float(v) for v in orientation)  # type: ignore[assignment]
        points = interpolate_polyline(stroke.points, self.point_spacing)
        if len(points) < 2:
            self.orientation = old_orientation
            return []
        result: List[PaperWaypoint] = [self._pose(points[0], self.lift_z, False)]
        result.append(self._pose(points[0], self.write_z, True))
        result.extend(self._pose(point, self.write_z, True) for point in points[1:])
        result.append(self._pose(points[-1], self.lift_z, False))
        self.orientation = old_orientation
        return result

    def build_sentence_path(
        self,
        strokes: Iterable[Stroke],
        orientation: Optional[Sequence[float]] = None,
    ) -> List[PaperWaypoint]:
        """Flatten all strokes into one Cartesian waypoint array.

        Every stroke remains separated by a lift waypoint.  MoveIt can thus
        attempt the whole sentence in one ``compute_cartesian_path`` call,
        while callers retain the original stroke list for fallback planning.
        """

        result: List[PaperWaypoint] = []
        for stroke in strokes:
            result.extend(self.stroke_to_waypoints(stroke, orientation))
        return result

    # Common aliases used by earlier prototypes.
    build_path = build_sentence_path
    build_waypoints = build_sentence_path

    def build_segmented_paths(self, strokes: Iterable[Stroke], orientation: Optional[Sequence[float]] = None) -> List[List[PaperWaypoint]]:
        """Return one lift/write/lift list per stroke for fallback planning."""

        return [segment for segment in (self.stroke_to_waypoints(stroke, orientation) for stroke in strokes) if segment]

    @staticmethod
    def to_pose(waypoint: PaperWaypoint, pose_type: Optional[Any] = None) -> Any:
        """Convert a paper waypoint to a ROS Pose when geometry_msgs exists.

        The fallback dictionary is useful for offline tests and is accepted by
        no MoveIt method; the executor always requests the real ROS type.
        """

        if pose_type is None:
            try:
                from geometry_msgs.msg import Pose  # type: ignore

                pose_type = Pose
            except Exception:
                pose_type = None
        if pose_type is None:
            return {
                "u": waypoint.u,
                "v": waypoint.v,
                "z": waypoint.z,
                "orientation": waypoint.orientation,
                "pen_down": waypoint.pen_down,
            }
        pose = pose_type()
        pose.position.x = float(waypoint.u)
        pose.position.y = float(-waypoint.v)
        pose.position.z = float(waypoint.z)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = waypoint.orientation
        return pose


__all__ = ["PaperWaypoint", "TrajectoryBuilder"]
