"""OpenCV line-art to :class:`~vla_writing.data_types.Stroke` conversion.

The parser intentionally performs classical image processing only: grayscale,
threshold/Canny, contour extraction, Douglas--Peucker simplification and
polyline resampling.  It is deterministic, has no network/model dependency,
and works equally well in a ROS node or in an offline unit test.
"""

from __future__ import annotations

import logging
import math
import os
from typing import Any, Iterable, List, Optional, Sequence, Tuple, Union

try:  # OpenCV is available in the Noetic desktop image, but keep imports lazy.
    import cv2  # type: ignore
except Exception:  # pragma: no cover - exercised only on minimal CI images
    cv2 = None

try:
    import numpy as np  # type: ignore
except Exception:  # pragma: no cover
    np = None

from .data_types import Glyph, Point2D, Stroke, coerce_point, coerce_stroke


LOG = logging.getLogger(__name__)
ArrayLike = Any


class ImageParser:
    """Extract and normalise contours from a black/white line drawing.

    Parameters mirror ``config/writing.yaml`` from the project specification.
    ``generate_strokes`` returns points normalised to a unit square; use
    :meth:`fit_to_box` when a caller wants physical paper coordinates directly.
    """

    def __init__(
        self,
        min_contour_length: float = 30.0,
        simplify_epsilon_ratio: float = 0.003,
        min_contour_area: float = 4.0,
        resample_spacing: Optional[float] = None,
        threshold: int = 0,
        use_canny: bool = False,
        blur_kernel: int = 3,
        invert: bool = True,
        retrieval_mode: Optional[int] = None,
    ):
        self.min_contour_length = float(min_contour_length)
        self.simplify_epsilon_ratio = float(simplify_epsilon_ratio)
        self.min_contour_area = float(min_contour_area)
        self.resample_spacing = None if resample_spacing is None else float(resample_spacing)
        self.threshold = int(threshold)
        self.use_canny = bool(use_canny)
        self.blur_kernel = max(1, int(blur_kernel) | 1)
        self.invert = bool(invert)
        # Thick black line art produces an inner and outer contour.  External
        # contours give one clean pen path by default; callers drawing filled
        # or multi-level artwork can request RETR_LIST/RETR_TREE explicitly.
        self.retrieval_mode = retrieval_mode

    @staticmethod
    def _require_cv() -> None:
        if cv2 is None or np is None:
            raise ImportError("ImageParser requires python3-opencv and numpy")

    def load_image(self, path_or_image: Union[str, os.PathLike, ArrayLike]) -> ArrayLike:
        """Load an image path or copy an already loaded numpy image."""

        self._require_cv()
        if isinstance(path_or_image, (str, bytes, os.PathLike)):
            image = cv2.imread(os.fspath(path_or_image), cv2.IMREAD_UNCHANGED)
            if image is None:
                raise IOError("Unable to read image: {}".format(path_or_image))
            return image
        if not hasattr(path_or_image, "shape"):
            raise TypeError("image must be a path or a numpy array")
        return np.array(path_or_image, copy=True)

    def preprocess(self, image: ArrayLike) -> ArrayLike:
        """Return a binary image in which foreground lines are 255."""

        self._require_cv()
        arr = image
        if len(arr.shape) == 3:
            if arr.shape[2] == 4:
                arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2GRAY)
            else:
                arr = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
        elif len(arr.shape) != 2:
            raise ValueError("image must have 2-D grayscale or 3-D BGR(A) shape")
        arr = np.asarray(arr, dtype=np.uint8)
        if self.blur_kernel > 1:
            arr = cv2.GaussianBlur(arr, (self.blur_kernel, self.blur_kernel), 0)
        if self.use_canny:
            # Let Canny derive sensible thresholds from image contrast.
            med = float(np.median(arr))
            low = int(max(0.0, 0.66 * med))
            high = int(min(255.0, max(low + 1, 1.33 * med)))
            binary = cv2.Canny(arr, low, high)
            return binary
        threshold = self.threshold
        if threshold <= 0:
            mode = cv2.THRESH_BINARY_INV if self.invert else cv2.THRESH_BINARY
            _, binary = cv2.threshold(arr, 0, 255, mode | cv2.THRESH_OTSU)
        else:
            mode = cv2.THRESH_BINARY_INV if self.invert else cv2.THRESH_BINARY
            _, binary = cv2.threshold(arr, threshold, 255, mode)
        return binary

    def extract_contours(self, image_or_binary: ArrayLike) -> List[ArrayLike]:
        """Extract candidate contours and filter tiny/noisy ones."""

        self._require_cv()
        arr = np.asarray(image_or_binary)
        if arr.dtype == np.bool_:
            arr = arr.astype(np.uint8) * 255
        if len(arr.shape) == 3 or (len(arr.shape) == 2 and arr.dtype != np.uint8):
            arr = self.preprocess(arr)
        elif len(arr.shape) == 2:
            # If this looks like a greyscale photograph rather than a binary
            # mask, preprocessing gives much better contours.
            unique = np.unique(arr)
            if len(unique) > 2:
                arr = self.preprocess(arr)
        mode = cv2.RETR_EXTERNAL if self.retrieval_mode is None else int(self.retrieval_mode)
        found = cv2.findContours(arr, mode, cv2.CHAIN_APPROX_NONE)
        contours = found[0] if len(found) == 2 else found[1]
        result: List[ArrayLike] = []
        for contour in contours:
            if contour is None or len(contour) < 2:
                continue
            length = float(cv2.arcLength(contour, False))
            closed_length = float(cv2.arcLength(contour, True))
            length = max(length, closed_length)
            if length < self.min_contour_length:
                continue
            area = abs(float(cv2.contourArea(contour)))
            # Open strokes often have near-zero area; retain them based on
            # length while removing isolated specks.
            if area < self.min_contour_area and length < 2.0 * self.min_contour_length:
                continue
            result.append(contour)
        return result

    def simplify_contours(self, contours: Iterable[ArrayLike]) -> List[ArrayLike]:
        self._require_cv()
        simplified: List[ArrayLike] = []
        for contour in contours:
            if contour is None or len(contour) < 2:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            epsilon = max(1e-6, self.simplify_epsilon_ratio * perimeter)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if approx is not None and len(approx) >= 2:
                simplified.append(approx)
        return simplified

    @staticmethod
    def _contour_points(contour: ArrayLike) -> List[Point2D]:
        if contour is None:
            return []
        arr = np.asarray(contour) if np is not None else contour
        points: List[Point2D] = []
        try:
            for item in arr.reshape((-1, 2)):
                p = Point2D(float(item[0]), float(item[1]))
                if not points or abs(points[-1].u - p.u) > 1e-9 or abs(points[-1].v - p.v) > 1e-9:
                    points.append(p)
        except Exception:
            for item in contour:
                try:
                    points.append(Point2D(float(item[0][0]), float(item[0][1])))
                except Exception:
                    continue
        return points

    @staticmethod
    def _distance(a: Point2D, b: Point2D) -> float:
        return math.hypot(a.u - b.u, a.v - b.v)

    @classmethod
    def _resample_polyline(cls, points: Sequence[Point2D], spacing: Optional[float]) -> List[Point2D]:
        if len(points) <= 1 or spacing is None or spacing <= 0:
            return [p.copy() for p in points]
        output = [points[0].copy()]
        for start, end in zip(points[:-1], points[1:]):
            dx = end.u - start.u
            dy = end.v - start.v
            distance = math.hypot(dx, dy)
            if distance <= 1e-12:
                continue
            count = max(1, int(math.ceil(distance / spacing)))
            for idx in range(1, count + 1):
                t = float(idx) / float(count)
                p = Point2D(start.u + t * dx, start.v + t * dy)
                if cls._distance(output[-1], p) > 1e-10:
                    output.append(p)
        return output

    @classmethod
    def optimize_order(
        cls,
        contours: Iterable[ArrayLike],
        start_point: Optional[Union[Point2D, Sequence[float]]] = None,
    ) -> List[List[Point2D]]:
        """Order contours by nearest-neighbour travel, reversing when useful."""

        pending = [cls._contour_points(c) for c in contours]
        pending = [c for c in pending if len(c) >= 2]
        ordered: List[List[Point2D]] = []
        current: Optional[Point2D] = None if start_point is None else coerce_point(start_point)
        while pending:
            if current is None:
                index = 0
                reverse = False
            else:
                best = float("inf")
                index = 0
                reverse = False
                for idx, contour in enumerate(pending):
                    d_start = cls._distance(current, contour[0])
                    d_end = cls._distance(current, contour[-1])
                    if d_start < best:
                        best, index, reverse = d_start, idx, False
                    if d_end < best:
                        best, index, reverse = d_end, idx, True
            contour = pending.pop(index)
            if reverse:
                contour.reverse()
            ordered.append(contour)
            current = contour[-1]
        return ordered

    @staticmethod
    def normalize_strokes(strokes: Iterable[Stroke], padding: float = 0.0) -> List[Stroke]:
        """Map stroke coordinates to a unit square while preserving aspect."""

        source = [coerce_stroke(s) for s in strokes]
        source = [s for s in source if len(s.points) >= 2]
        if not source:
            return []
        points = [p for stroke in source for p in stroke.points]
        min_u = min(p.u for p in points)
        max_u = max(p.u for p in points)
        min_v = min(p.v for p in points)
        max_v = max(p.v for p in points)
        span_u = max(max_u - min_u, 1e-12)
        span_v = max(max_v - min_v, 1e-12)
        pad = max(0.0, min(0.49, float(padding)))
        scale = (1.0 - 2.0 * pad) / max(span_u, span_v)
        # Center the shorter dimension rather than stretching the drawing.
        used_u = span_u * scale
        used_v = span_v * scale
        offset_u = pad + 0.5 * ((1.0 - 2.0 * pad) - used_u)
        offset_v = pad + 0.5 * ((1.0 - 2.0 * pad) - used_v)
        result: List[Stroke] = []
        for stroke in source:
            result.append(
                Stroke(
                    [Point2D((p.u - min_u) * scale + offset_u, (p.v - min_v) * scale + offset_v) for p in stroke.points],
                    stroke.closed,
                    dict(stroke.metadata),
                )
            )
        return result

    @staticmethod
    def fit_to_box(
        strokes: Iterable[Stroke],
        width: float,
        height: float,
        origin_u: float = 0.0,
        origin_v: float = 0.0,
        padding: float = 0.0,
    ) -> List[Stroke]:
        """Scale normalised strokes into a physical paper rectangle."""

        normal = ImageParser.normalize_strokes(strokes, padding=padding)
        return [
            Stroke(
                [Point2D(origin_u + p.u * float(width), origin_v + p.v * float(height)) for p in stroke.points],
                stroke.closed,
                dict(stroke.metadata),
            )
            for stroke in normal
        ]

    def generate_strokes(
        self,
        path_or_image: Union[str, os.PathLike, ArrayLike],
        normalize: bool = True,
        padding: float = 0.02,
        close_contours: bool = True,
    ) -> List[Stroke]:
        """Run the complete image-to-strokes pipeline."""

        image = self.load_image(path_or_image)
        binary = self.preprocess(image)
        contours = self.extract_contours(binary)
        contours = self.simplify_contours(contours)
        ordered = self.optimize_order(contours)
        strokes: List[Stroke] = []
        for points in ordered:
            if close_contours and len(points) >= 3:
                if self._distance(points[0], points[-1]) > 1e-9:
                    points = points + [points[0].copy()]
                closed = True
            else:
                closed = False
            # If ``normalize`` is requested, resample after mapping to the
            # unit square.  This makes the config's 0.002 value mean roughly
            # 2 mm of the eventual paper, rather than 0.002 source pixels
            # (which would create hundreds of thousands of points).
            raw_spacing = None if normalize else self.resample_spacing
            points = self._resample_polyline(points, raw_spacing)
            if len(points) >= 2:
                strokes.append(Stroke(points, closed, {"source": "opencv_contour"}))
        if not normalize:
            return strokes
        normal = self.normalize_strokes(strokes, padding=padding)
        if self.resample_spacing is None or self.resample_spacing <= 0:
            return normal
        return [
            Stroke(
                self._resample_polyline(stroke.points, self.resample_spacing),
                stroke.closed,
                dict(stroke.metadata),
            )
            for stroke in normal
        ]

    def image_to_glyph(
        self,
        path_or_image: Union[str, os.PathLike, ArrayLike],
        symbol: str = "<image>",
        padding: float = 0.02,
    ) -> Glyph:
        strokes = self.generate_strokes(path_or_image, normalize=True, padding=padding)
        if not strokes:
            return Glyph(symbol, [], 1.0, 1.0, {"source": "opencv_contour"})
        return Glyph(symbol, strokes, 1.0, 1.0, {"source": "opencv_contour", "image": True})


__all__ = ["ImageParser"]
