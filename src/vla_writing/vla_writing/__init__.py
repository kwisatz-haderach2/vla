"""ROS-independent building blocks for the UR5e writing demo."""

# Catkin generates ``vla_writing.srv`` into a second overlay directory.  Make
# this source package path-extensible so the generated service package remains
# importable when the source checkout is ahead of ``devel`` on PYTHONPATH.
from pkgutil import extend_path
__path__ = extend_path(__path__, __name__)

from .data_types import Glyph, Point2D, Stroke
from .ascii_font import AsciiFont
from .hanzi_parser import HanziParser
from .image_parser import ImageParser
from .layout_engine import LayoutEngine
from .trajectory_builder import PaperWaypoint, TrajectoryBuilder
from .interpolator import (
    interpolate_polyline,
    interpolate_stroke,
    interpolate_strokes,
    polyline_length,
)
from .joint_state_utils import (
    UR5E_JOINT_LIMITS,
    canonical_angle,
    canonicalize_joint_state,
    invalid_joint_state,
    normalize_joint_positions,
    normalize_trajectory,
    wrap_to_pi,
)

__all__ = [
    "Glyph", "Point2D", "Stroke", "AsciiFont", "HanziParser",
    "ImageParser", "LayoutEngine", "PaperWaypoint", "TrajectoryBuilder",
    "interpolate_polyline", "interpolate_stroke", "interpolate_strokes",
    "polyline_length", "UR5E_JOINT_LIMITS", "wrap_to_pi",
    "canonical_angle", "canonicalize_joint_state", "invalid_joint_state",
    "normalize_joint_positions", "normalize_trajectory",
]
