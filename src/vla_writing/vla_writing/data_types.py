"""Data structures shared by the text, image and motion pipelines.

The parsers in :mod:`vla_writing` deliberately do not depend on ROS.  Keeping
the intermediate representation small makes it possible to unit test the
trajectory generation on a machine without Gazebo/MoveIt installed and also
gives the ROS1 and ROS2 parts of the project a stable interface.

Coordinates in a :class:`Point2D` are *paper logical coordinates*: ``u``
increases to the right and ``v`` increases downwards.  Values are normally in
metres after layout (font and image parsers may use normalised 0..1 values
until then).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union


Number = Union[int, float]
PointLike = Union["Point2D", Sequence[Number], Mapping[str, Number]]


@dataclass
class Point2D:
    """A point in the paper's two-dimensional logical coordinate system."""

    u: float
    v: float

    def __post_init__(self) -> None:
        self.u = float(self.u)
        self.v = float(self.v)

    def as_tuple(self) -> Tuple[float, float]:
        return (self.u, self.v)

    def copy(self) -> "Point2D":
        return Point2D(self.u, self.v)

    def __iter__(self):
        yield self.u
        yield self.v

    def __add__(self, other: PointLike) -> "Point2D":
        p = coerce_point(other)
        return Point2D(self.u + p.u, self.v + p.v)

    def __sub__(self, other: PointLike) -> "Point2D":
        p = coerce_point(other)
        return Point2D(self.u - p.u, self.v - p.v)

    def scaled(self, sx: Number, sy: Optional[Number] = None) -> "Point2D":
        """Return a scaled copy (``sy`` defaults to ``sx``)."""

        if sy is None:
            sy = sx
        return Point2D(self.u * float(sx), self.v * float(sy))


@dataclass
class Stroke:
    """One continuous pen-down polyline.

    ``closed`` is a hint used by the image parser.  The points are not
    implicitly closed; callers that need a closing segment should append the
    first point explicitly.  ``metadata`` is intentionally free-form so a
    renderer can preserve source information without changing this interface.
    """

    points: List[Point2D] = field(default_factory=list)
    closed: bool = False
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # Permit a JSON-style stroke record (``{"points": [...]}``) in
        # addition to a bare point sequence.
        if isinstance(self.points, Mapping):
            self.points = self.points.get("points", [])  # type: ignore[assignment]
        self.points = [coerce_point(p) for p in self.points]

    def copy(self) -> "Stroke":
        return Stroke([p.copy() for p in self.points], self.closed, dict(self.metadata))

    def is_empty(self) -> bool:
        return len(self.points) == 0

    def __len__(self) -> int:
        return len(self.points)


@dataclass
class Glyph:
    """A character/image symbol represented by one or more local strokes.

    ``width`` and ``height`` are local advance dimensions.  Font glyphs use
    values around 1.0, while a parser may provide a different aspect ratio;
    :class:`LayoutEngine` scales them to the configured physical size.
    """

    symbol: str
    strokes: List[Stroke] = field(default_factory=list)
    width: float = 1.0
    height: float = 1.0
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.symbol = str(self.symbol)
        self.strokes = [coerce_stroke(s) for s in self.strokes]
        self.width = float(self.width)
        self.height = float(self.height)

    @property
    def supported(self) -> bool:
        """Whether the glyph contains drawable data.

        A missing Hanzi is represented by an empty glyph instead of raising an
        exception.  This property lets clients distinguish that case while
        keeping the common ``Glyph`` return type.
        """

        return any(not stroke.is_empty() for stroke in self.strokes)

    def copy(self) -> "Glyph":
        return Glyph(
            self.symbol,
            [s.copy() for s in self.strokes],
            self.width,
            self.height,
            dict(self.metadata),
        )


def coerce_point(value: PointLike) -> Point2D:
    """Convert common point representations to :class:`Point2D`.

    Accepted forms are ``Point2D``, ``(u, v)``/``[u, v]`` and mappings with
    either ``u``/``v`` or ``x``/``y`` keys.  A clear ``ValueError`` is raised
    for malformed input so parser errors are easy to diagnose.
    """

    if isinstance(value, Point2D):
        return value.copy()
    if isinstance(value, Mapping):
        if "u" in value and "v" in value:
            return Point2D(value["u"], value["v"])
        if "x" in value and "y" in value:
            return Point2D(value["x"], value["y"])
        raise ValueError("point mapping must contain u/v or x/y")
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        if len(value) < 2:
            raise ValueError("point sequence must contain at least two values")
        return Point2D(value[0], value[1])
    raise ValueError("unsupported point representation: {!r}".format(value))


def coerce_stroke(value: Union[Stroke, Iterable[PointLike]]) -> Stroke:
    if isinstance(value, Stroke):
        return value.copy()
    if isinstance(value, Mapping):
        return Stroke(value.get("points", []), bool(value.get("closed", False)), dict(value.get("metadata", {})))
    return Stroke([coerce_point(p) for p in value])


def clone_strokes(strokes: Iterable[Union[Stroke, Iterable[PointLike]]]) -> List[Stroke]:
    """Deep-copy a stroke sequence while accepting tuple/list input."""

    return [coerce_stroke(stroke) for stroke in strokes]


__all__ = [
    "Point2D",
    "Stroke",
    "Glyph",
    "PointLike",
    "coerce_point",
    "coerce_stroke",
    "clone_strokes",
]
