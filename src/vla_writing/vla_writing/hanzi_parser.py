"""Make Me a Hanzi ``graphics.txt`` reader.

Only the ``medians`` (centre lines) are used for motion.  The source database
is a line-oriented UTF-8 file, usually with one of these forms::

    你\t{"strokes": [...], "medians": [[[...], ...], ...]}
    你 {"medians": [[[...], ...], ...]}

The parser accepts both forms, tolerates a UTF-8 BOM and ignores malformed
records.  Loading is lazy and cached, which keeps ROS node start-up fast when
only English text is being demonstrated.
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .data_types import Glyph, Point2D, Stroke


LOG = logging.getLogger(__name__)


def _fallback_glyph_data() -> Dict[str, List[List[List[float]]]]:
    """Small emergency set used when graphics.txt is not installed.

    It is intentionally only a safety net; a full Make Me a Hanzi database
    should be distributed in ``data/hanzi/graphics.txt`` for production use.
    The coordinates are already normalised to a unit square.
    """

    def l(*p):
        return [[float(x), float(y)] for x, y in p]

    return {
        "一": [l((0.08, 0.50), (0.92, 0.50))],
        "丨": [l((0.50, 0.08), (0.50, 0.92))],
        "十": [l((0.08, 0.50), (0.92, 0.50)), l((0.50, 0.08), (0.50, 0.92))],
        "人": [l((0.50, 0.08), (0.12, 0.92)), l((0.50, 0.08), (0.88, 0.92))],
        "大": [l((0.50, 0.08), (0.50, 0.92)), l((0.50, 0.34), (0.10, 0.82)), l((0.50, 0.34), (0.90, 0.82))],
        "中": [l((0.50, 0.08), (0.50, 0.92)), l((0.15, 0.25), (0.85, 0.25), (0.85, 0.75), (0.15, 0.75), (0.15, 0.25))],
        "口": [l((0.15, 0.20), (0.85, 0.20), (0.85, 0.80), (0.15, 0.80), (0.15, 0.20))],
        "日": [l((0.15, 0.12), (0.85, 0.12), (0.85, 0.88), (0.15, 0.88), (0.15, 0.12)), l((0.15, 0.50), (0.85, 0.50))],
        "你": [l((0.25, 0.12), (0.15, 0.88)), l((0.25, 0.12), (0.42, 0.30)), l((0.42, 0.30), (0.20, 0.45)), l((0.42, 0.30), (0.42, 0.88)), l((0.62, 0.22), (0.82, 0.22), (0.82, 0.78), (0.62, 0.78), (0.62, 0.22)), l((0.62, 0.50), (0.82, 0.50))],
        "好": [l((0.25, 0.10), (0.25, 0.90)), l((0.08, 0.35), (0.45, 0.35)), l((0.08, 0.68), (0.45, 0.68)), l((0.72, 0.10), (0.72, 0.90)), l((0.55, 0.35), (0.92, 0.35)), l((0.55, 0.68), (0.92, 0.68))],
        "机": [l((0.20, 0.10), (0.20, 0.90)), l((0.08, 0.35), (0.42, 0.35)), l((0.08, 0.68), (0.42, 0.68)), l((0.62, 0.10), (0.62, 0.90)), l((0.50, 0.30), (0.90, 0.30)), l((0.50, 0.65), (0.90, 0.65))],
        "器": [l((0.12, 0.15), (0.42, 0.15), (0.42, 0.40), (0.12, 0.40), (0.12, 0.15)), l((0.58, 0.15), (0.88, 0.15), (0.88, 0.40), (0.58, 0.40), (0.58, 0.15)), l((0.12, 0.60), (0.42, 0.60), (0.42, 0.85), (0.12, 0.85), (0.12, 0.60)), l((0.58, 0.60), (0.88, 0.60), (0.88, 0.85), (0.58, 0.85), (0.58, 0.60)), l((0.50, 0.05), (0.50, 0.95))],
        "我": [l((0.18, 0.10), (0.18, 0.90)), l((0.05, 0.35), (0.42, 0.35)), l((0.05, 0.68), (0.42, 0.68)), l((0.60, 0.12), (0.60, 0.88)), l((0.50, 0.30), (0.92, 0.30)), l((0.50, 0.62), (0.92, 0.62))],
        "们": [l((0.22, 0.12), (0.22, 0.90)), l((0.08, 0.35), (0.42, 0.35)), l((0.08, 0.68), (0.42, 0.68)), l((0.58, 0.10), (0.58, 0.90)), l((0.58, 0.25), (0.88, 0.25), (0.88, 0.75), (0.58, 0.75))],
        "书": [l((0.18, 0.18), (0.82, 0.18)), l((0.50, 0.08), (0.50, 0.92)), l((0.18, 0.50), (0.82, 0.50)), l((0.18, 0.82), (0.82, 0.82))],
        "写": [l((0.15, 0.20), (0.85, 0.20)), l((0.50, 0.10), (0.50, 0.90)), l((0.15, 0.50), (0.85, 0.50)), l((0.15, 0.82), (0.85, 0.82))],
        "字": [l((0.15, 0.20), (0.85, 0.20)), l((0.50, 0.10), (0.50, 0.38)), l((0.20, 0.50), (0.80, 0.50)), l((0.20, 0.50), (0.20, 0.82), (0.80, 0.82), (0.80, 0.50))],
        "画": [l((0.12, 0.12), (0.88, 0.12), (0.88, 0.88), (0.12, 0.88), (0.12, 0.12)), l((0.12, 0.50), (0.88, 0.50)), l((0.50, 0.12), (0.50, 0.88))],
    }


class HanziParser:
    """Lazy reader and normaliser for Make Me a Hanzi medians data."""

    def __init__(
        self,
        graphics_file: Optional[str] = None,
        canvas_size: float = 1024.0,
        baseline: float = 900.0,
        use_fallback: bool = True,
        warn_missing: bool = True,
    ):
        package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
        if graphics_file and not os.path.isabs(os.fspath(graphics_file)):
            package_relative = os.path.join(package_root, os.fspath(graphics_file))
            self.graphics_file = package_relative if os.path.isfile(package_relative) else os.fspath(graphics_file)
        else:
            self.graphics_file = graphics_file or os.path.join(package_root, "data", "hanzi", "graphics.txt")
        self.canvas_size = float(canvas_size)
        self.baseline = float(baseline)
        self.use_fallback = bool(use_fallback)
        self.warn_missing = bool(warn_missing)
        self.database: Dict[str, Any] = {}
        self._cache: Dict[str, Glyph] = {}
        self._loaded = False
        self._fallbacks = _fallback_glyph_data()

    @staticmethod
    def is_hanzi(char: str) -> bool:
        """Return true for common CJK Unified Ideographs and extensions."""

        if not char:
            return False
        code = ord(char[0])
        return (
            0x3400 <= code <= 0x4DBF
            or 0x4E00 <= code <= 0x9FFF
            or 0xF900 <= code <= 0xFAFF
            or 0x20000 <= code <= 0x2FA1F
        )

    def load_database(self) -> Dict[str, Any]:
        if self._loaded:
            return self.database
        self.database = {}
        if os.path.isfile(self.graphics_file):
            try:
                with open(self.graphics_file, "r", encoding="utf-8-sig") as handle:
                    for line_number, line in enumerate(handle, 1):
                        parsed = self._parse_line(line)
                        if parsed is None:
                            continue
                        char, record = parsed
                        self.database[char] = record
            except (OSError, UnicodeError) as exc:
                LOG.warning("Could not read Hanzi database %s: %s", self.graphics_file, exc)
        else:
            LOG.warning(
                "Hanzi graphics file not found at %s. Install Make Me a Hanzi data "
                "file (preserving its license); built-in demo fallbacks will be used.",
                self.graphics_file,
            )
        self._loaded = True
        return self.database

    @staticmethod
    def _parse_line(line: str) -> Optional[Tuple[str, Any]]:
        line = line.strip().lstrip("\ufeff")
        if not line or line.startswith("#"):
            return None
        # Current Make Me a Hanzi releases use one JSON object per line with a
        # ``character`` member.  Handle this first because the JSON delimiter
        # is at column zero (the legacy prefix parser below intentionally
        # requires a character before the delimiter).
        if line.startswith("{"):
            try:
                record = json.loads(line)
            except (TypeError, ValueError):
                return None
            if isinstance(record, Mapping):
                char = record.get("character", record.get("char"))
                if isinstance(char, str) and char:
                    return char[0], record
            return None
        # Most releases separate the character and JSON with a tab; finding
        # the first JSON delimiter also handles one-space and JSONL variants.
        start = -1
        for delimiter in ("{", "["):
            idx = line.find(delimiter)
            if idx >= 0 and (start < 0 or idx < start):
                start = idx
        if start < 0:
            return None
        try:
            record = json.loads(line[start:])
        except (TypeError, ValueError):
            return None
        # Current Make Me a Hanzi releases use JSONL records such as
        # {"character":"你", "medians":[...]}; older releases prefix the
        # JSON with the character separated by whitespace.  Accept both.
        if isinstance(record, Mapping) and record.get("character"):
            char = str(record["character"])[0]
        else:
            prefix = line[:start].strip()
            if not prefix:
                return None
            char = prefix[0]
        return char, record

    @staticmethod
    def _point_from_raw(raw: Any) -> Optional[Point2D]:
        try:
            if isinstance(raw, Mapping):
                if "x" in raw and "y" in raw:
                    return Point2D(raw["x"], raw["y"])
                if "u" in raw and "v" in raw:
                    return Point2D(raw["u"], raw["v"])
            if (isinstance(raw, Sequence) or hasattr(raw, "__len__")) and not isinstance(raw, (str, bytes)) and len(raw) >= 2:
                return Point2D(raw[0], raw[1])
        except (TypeError, ValueError):
            return None
        return None

    def convert_medians_to_strokes(self, medians: Any) -> List[Stroke]:
        """Convert raw 1024x900 medians to normalised paper coordinates."""

        if (not isinstance(medians, Sequence) and not hasattr(medians, "__iter__")) or isinstance(medians, (str, bytes)):
            return []
        result: List[Stroke] = []
        for raw_stroke in medians:
            if (not isinstance(raw_stroke, Sequence) and not hasattr(raw_stroke, "__iter__")) or isinstance(raw_stroke, (str, bytes)):
                continue
            points: List[Point2D] = []
            for raw_point in raw_stroke:
                point = self._point_from_raw(raw_point)
                if point is None:
                    continue
                # Make Me a Hanzi uses x∈[0,1024], y∈[0,900].  Keep a tiny
                # amount of overshoot (e.g. descenders) but avoid pathological
                # records from corrupt files.
                u = point.u / self.canvas_size
                v = (self.baseline - point.v) / self.canvas_size
                if points and abs(points[-1].u - u) < 1e-9 and abs(points[-1].v - v) < 1e-9:
                    continue
                points.append(Point2D(u, v))
            if len(points) >= 2:
                result.append(Stroke(points))
        return result

    def get_glyph(self, char: str) -> Glyph:
        self.load_database()
        if not char:
            return Glyph("", [], 1.0, 1.0)
        ch = char[0]
        if ch in self._cache:
            return self._cache[ch].copy()
        record = self.database.get(ch)
        source = "graphics.txt"
        if isinstance(record, Mapping):
            medians = record.get("medians", [])
        elif isinstance(record, Sequence):
            medians = record
        else:
            medians = []
        strokes = self.convert_medians_to_strokes(medians)
        if not strokes and self.use_fallback and ch in self._fallbacks:
            strokes = [Stroke([Point2D(p[0], p[1]) for p in raw]) for raw in self._fallbacks[ch]]
            source = "built-in-fallback"
        if not strokes and self.warn_missing and self.is_hanzi(ch):
            LOG.warning("No medians available for Hanzi %s; skipping this glyph", ch)
        metadata = {"source": source, "hanzi": True}
        if not strokes:
            metadata["missing"] = True
        glyph = Glyph(ch, strokes, 1.0, 1.0, metadata)
        self._cache[ch] = glyph
        return glyph.copy()

    def get_medians(self, char: str) -> List[Stroke]:
        """Return only the normalised median strokes for ``char``."""

        return self.get_glyph(char).strokes

    def has_glyph(self, char: str) -> bool:
        self.load_database()
        return bool(char) and (char[0] in self.database or (self.use_fallback and char[0] in self._fallbacks))

    def glyphs_for_text(self, text: str) -> List[Glyph]:
        return [self.get_glyph(ch) for ch in str(text)]

    # Friendly aliases used by a few Make Me a Hanzi examples.
    convert_medians = convert_medians_to_strokes


__all__ = ["HanziParser"]
