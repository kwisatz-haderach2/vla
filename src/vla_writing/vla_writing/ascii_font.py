"""A dependency-free single-line ASCII vector font.

The normal data source is ``data/ascii_single_line.json``.  A built-in copy of
the compact vector definitions is kept as a fallback so the demo still works
when a package is installed without its data directory (a surprisingly common
failure mode with catkin overlays).  Coordinates are normalised to a 1x1
character box and use the same downwards ``v`` direction as the paper layout.
"""

from __future__ import annotations

import json
import logging
import math
import os
import string
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

from .data_types import Glyph, Point2D, Stroke


LOG = logging.getLogger(__name__)

_FULLWIDTH_ALIASES = {
    "，": ",",
    "。": ".",
    "！": "!",
    "？": "?",
    "：": ":",
    "；": ";",
    "（": "(",
    "）": ")",
    "【": "[",
    "】": "]",
    "“": '"',
    "”": '"',
    "‘": "'",
    "’": "'",
    "－": "-",
}


def _line(*points: Sequence[float]) -> List[List[float]]:
    return [[float(p[0]), float(p[1])] for p in points]


def _poly(*points: Sequence[float]) -> List[List[float]]:
    return _line(*points)


def _box(x0: float = 0.08, y0: float = 0.08, x1: float = 0.92, y1: float = 0.92) -> List[List[List[float]]]:
    return [
        _line((x0, y0), (x1, y0)),
        _line((x1, y0), (x1, y1)),
        _line((x1, y1), (x0, y1)),
        _line((x0, y1), (x0, y0)),
    ]


def _builtin_font_data() -> Dict[str, Dict[str, Any]]:
    """Return the complete printable ASCII fallback font.

    The definitions intentionally use simple centre-line strokes rather than
    filled outlines.  This keeps the number of Cartesian waypoints manageable
    while remaining legible on the simulated paper.
    """

    s: Dict[str, Dict[str, Any]] = {}

    def add(ch: str, strokes: Iterable[Iterable[Sequence[float]]], width: float = 1.0) -> None:
        s[ch] = {"strokes": [[list(map(float, p)) for p in st] for st in strokes], "width": width, "height": 1.0}

    # Uppercase letters.
    add("A", [_line((0.05, 1), (0.50, 0)), _line((0.50, 0), (0.95, 1)), _line((0.23, 0.58), (0.77, 0.58))])
    add("B", [_line((0.10, 0), (0.10, 1)), _poly((0.10, 0), (0.58, 0), (0.84, 0.12), (0.84, 0.38), (0.58, 0.50), (0.10, 0.50)), _poly((0.10, 0.50), (0.58, 0.50), (0.84, 0.62), (0.84, 0.88), (0.58, 1), (0.10, 1))])
    add("C", [_poly((0.90, 0.12), (0.72, 0.02), (0.30, 0.02), (0.08, 0.18), (0.08, 0.82), (0.30, 0.98), (0.72, 0.98), (0.90, 0.88))])
    add("D", [_line((0.10, 0), (0.10, 1)), _poly((0.10, 0), (0.55, 0), (0.90, 0.18), (0.90, 0.82), (0.55, 1), (0.10, 1))])
    add("E", [_line((0.90, 0), (0.10, 0), (0.10, 1), (0.90, 1)), _line((0.10, 0.50), (0.72, 0.50))])
    add("F", [_line((0.10, 1), (0.10, 0), (0.90, 0)), _line((0.10, 0.50), (0.72, 0.50))])
    add("G", [_poly((0.90, 0.18), (0.72, 0.03), (0.28, 0.03), (0.08, 0.18), (0.08, 0.82), (0.28, 0.97), (0.72, 0.97), (0.90, 0.82), (0.90, 0.58), (0.52, 0.58))])
    add("H", [_line((0.10, 0), (0.10, 1)), _line((0.90, 0), (0.90, 1)), _line((0.10, 0.50), (0.90, 0.50))])
    add("I", [_line((0.10, 0), (0.90, 0)), _line((0.50, 0), (0.50, 1)), _line((0.10, 1), (0.90, 1))])
    add("J", [_poly((0.10, 0), (0.90, 0)), _line((0.70, 0), (0.70, 0.82)), _poly((0.70, 0.82), (0.52, 0.98), (0.25, 0.98), (0.08, 0.82))])
    add("K", [_line((0.10, 0), (0.10, 1)), _line((0.90, 0), (0.10, 0.50), (0.90, 1))])
    add("L", [_line((0.10, 0), (0.10, 1), (0.90, 1))])
    add("M", [_line((0.08, 1), (0.08, 0)), _poly((0.08, 0), (0.50, 0.52), (0.92, 0)), _line((0.92, 0), (0.92, 1))])
    add("N", [_line((0.10, 1), (0.10, 0)), _line((0.10, 0), (0.90, 1)), _line((0.90, 1), (0.90, 0))])
    add("O", [_poly((0.28, 0.03), (0.72, 0.03), (0.92, 0.20), (0.92, 0.80), (0.72, 0.97), (0.28, 0.97), (0.08, 0.80), (0.08, 0.20), (0.28, 0.03))])
    add("P", [_line((0.10, 1), (0.10, 0)), _poly((0.10, 0), (0.58, 0), (0.86, 0.14), (0.86, 0.36), (0.58, 0.50), (0.10, 0.50))])
    add("Q", [_poly((0.28, 0.03), (0.72, 0.03), (0.92, 0.20), (0.92, 0.80), (0.72, 0.97), (0.28, 0.97), (0.08, 0.80), (0.08, 0.20), (0.28, 0.03)), _line((0.58, 0.68), (0.92, 1.00))])
    add("R", [_line((0.10, 1), (0.10, 0)), _poly((0.10, 0), (0.58, 0), (0.86, 0.14), (0.86, 0.36), (0.58, 0.50), (0.10, 0.50)), _line((0.56, 0.50), (0.92, 1))])
    add("S", [_poly((0.88, 0.14), (0.68, 0.03), (0.28, 0.03), (0.08, 0.18), (0.08, 0.38), (0.28, 0.50), (0.70, 0.50), (0.90, 0.64), (0.90, 0.84), (0.70, 0.97), (0.28, 0.97), (0.08, 0.84))])
    add("T", [_line((0.08, 0), (0.92, 0)), _line((0.50, 0), (0.50, 1))])
    add("U", [_poly((0.10, 0), (0.10, 0.80), (0.28, 0.97), (0.72, 0.97), (0.90, 0.80), (0.90, 0))])
    add("V", [_line((0.08, 0), (0.50, 1), (0.92, 0))])
    add("W", [_poly((0.05, 0), (0.27, 1), (0.50, 0.52), (0.73, 1), (0.95, 0))])
    add("X", [_line((0.08, 0), (0.92, 1)), _line((0.92, 0), (0.08, 1))])
    add("Y", [_poly((0.08, 0), (0.50, 0.48), (0.92, 0)), _line((0.50, 0.48), (0.50, 1))])
    add("Z", [_line((0.08, 0), (0.92, 0), (0.08, 1), (0.92, 1))])

    # Lowercase letters.  These are deliberately compact and use a 0.78
    # x-height; ascenders/descenders make mixed-case text easier to read.
    add("a", [_poly((0.12, 0.58), (0.28, 0.45), (0.65, 0.45), (0.80, 0.58), (0.80, 1), (0.62, 1), (0.12, 1), (0.12, 0.58)), _line((0.80, 0.58), (0.80, 1))])
    add("b", [_line((0.14, 0), (0.14, 1)), _poly((0.14, 0.48), (0.34, 0.44), (0.70, 0.48), (0.84, 0.64), (0.84, 0.84), (0.70, 1), (0.34, 1), (0.14, 0.92))])
    add("c", [_poly((0.82, 0.56), (0.65, 0.45), (0.30, 0.45), (0.12, 0.60), (0.12, 0.85), (0.30, 1), (0.65, 1), (0.82, 0.90))])
    add("d", [_line((0.82, 0), (0.82, 1)), _poly((0.82, 0.48), (0.62, 0.44), (0.28, 0.48), (0.12, 0.64), (0.12, 0.84), (0.28, 1), (0.62, 1), (0.82, 0.92))])
    add("e", [_poly((0.12, 0.70), (0.80, 0.70), (0.70, 0.50), (0.30, 0.45), (0.12, 0.62), (0.12, 0.86), (0.30, 1), (0.68, 1), (0.82, 0.90))])
    add("f", [_poly((0.78, 0.12), (0.62, 0.02), (0.42, 0.12), (0.42, 1)), _line((0.20, 0.46), (0.72, 0.46))])
    add("g", [_poly((0.80, 0.56), (0.64, 0.45), (0.28, 0.45), (0.12, 0.60), (0.12, 0.84), (0.28, 1), (0.64, 1), (0.80, 0.88)), _poly((0.80, 0.55), (0.80, 0.96), (0.64, 1.0), (0.28, 1.0), (0.12, 0.92))])
    add("h", [_line((0.14, 0), (0.14, 1)), _poly((0.14, 0.58), (0.35, 0.45), (0.66, 0.45), (0.82, 0.60), (0.82, 1))])
    add("i", [_line((0.50, 0.45), (0.50, 1)), _line((0.50, 0.10), (0.50, 0.12))])
    add("j", [_poly((0.62, 0.45), (0.62, 0.96), (0.48, 1.0), (0.24, 1.0), (0.10, 0.92)), _line((0.62, 0.10), (0.62, 0.12))])
    add("k", [_line((0.14, 0), (0.14, 1)), _line((0.82, 0.45), (0.14, 0.72), (0.82, 1))])
    add("l", [_line((0.50, 0), (0.50, 1))])
    add("m", [_line((0.10, 1), (0.10, 0.48)), _poly((0.10, 0.58), (0.26, 0.45), (0.44, 0.58), (0.44, 1)), _poly((0.44, 0.58), (0.60, 0.45), (0.78, 0.58), (0.78, 1))])
    add("n", [_line((0.12, 1), (0.12, 0.48)), _poly((0.12, 0.58), (0.32, 0.45), (0.62, 0.45), (0.80, 0.60), (0.80, 1))])
    add("o", [_poly((0.30, 0.45), (0.66, 0.45), (0.82, 0.62), (0.82, 0.84), (0.66, 1), (0.30, 1), (0.12, 0.84), (0.12, 0.62), (0.30, 0.45))])
    add("p", [_line((0.14, 0.45), (0.14, 1.0)), _poly((0.14, 0.56), (0.34, 0.45), (0.68, 0.45), (0.84, 0.62), (0.84, 0.84), (0.68, 1), (0.34, 1), (0.14, 0.90))])
    add("q", [_line((0.82, 0.45), (0.82, 1.0)), _poly((0.82, 0.56), (0.62, 0.45), (0.28, 0.45), (0.12, 0.62), (0.12, 0.84), (0.28, 1), (0.62, 1), (0.82, 0.90))])
    add("r", [_line((0.14, 1), (0.14, 0.45)), _poly((0.14, 0.62), (0.35, 0.45), (0.72, 0.45), (0.84, 0.58))])
    add("s", [_poly((0.80, 0.55), (0.62, 0.45), (0.28, 0.45), (0.12, 0.60), (0.28, 0.72), (0.66, 0.72), (0.82, 0.86), (0.66, 1), (0.28, 1), (0.12, 0.90))])
    add("t", [_line((0.50, 0.18), (0.50, 1)), _line((0.22, 0.45), (0.78, 0.45)), _poly((0.50, 1), (0.68, 1), (0.80, 0.90))])
    add("u", [_line((0.14, 0.45), (0.14, 0.84)), _poly((0.14, 0.84), (0.30, 1), (0.62, 1), (0.80, 0.84)), _line((0.80, 0.45), (0.80, 1))])
    add("v", [_line((0.10, 0.45), (0.48, 1), (0.86, 0.45))])
    add("w", [_poly((0.08, 0.45), (0.27, 1), (0.50, 0.62), (0.73, 1), (0.92, 0.45))])
    add("x", [_line((0.12, 0.45), (0.82, 1)), _line((0.82, 0.45), (0.12, 1))])
    add("y", [_poly((0.10, 0.45), (0.48, 0.88), (0.84, 0.45)), _poly((0.84, 0.45), (0.70, 0.86), (0.50, 1.0), (0.24, 1.0), (0.10, 0.92))])
    add("z", [_line((0.12, 0.45), (0.84, 0.45), (0.12, 1), (0.84, 1))])

    # Digits.
    add("0", [_poly((0.28, 0.03), (0.72, 0.03), (0.90, 0.20), (0.90, 0.80), (0.72, 0.97), (0.28, 0.97), (0.10, 0.80), (0.10, 0.20), (0.28, 0.03))])
    add("1", [_line((0.25, 0.20), (0.50, 0.03), (0.50, 0.97)), _line((0.22, 0.97), (0.78, 0.97))])
    add("2", [_poly((0.12, 0.22), (0.28, 0.04), (0.70, 0.04), (0.88, 0.20), (0.88, 0.36), (0.12, 0.97), (0.88, 0.97))])
    add("3", [_poly((0.12, 0.12), (0.30, 0.04), (0.70, 0.04), (0.88, 0.18), (0.70, 0.50), (0.88, 0.82), (0.70, 0.97), (0.30, 0.97), (0.12, 0.88)), _line((0.40, 0.50), (0.70, 0.50))])
    add("4", [_poly((0.72, 0.97), (0.72, 0.04), (0.12, 0.68), (0.90, 0.68))])
    add("5", [_poly((0.86, 0.04), (0.18, 0.04), (0.12, 0.50), (0.68, 0.50), (0.88, 0.66), (0.88, 0.84), (0.70, 0.97), (0.28, 0.97), (0.10, 0.84))])
    add("6", [_poly((0.82, 0.12), (0.68, 0.04), (0.30, 0.04), (0.10, 0.22), (0.10, 0.80), (0.28, 0.97), (0.68, 0.97), (0.88, 0.80), (0.88, 0.64), (0.68, 0.50), (0.10, 0.64))])
    add("7", [_line((0.10, 0.04), (0.90, 0.04), (0.32, 0.97))])
    add("8", [_poly((0.28, 0.50), (0.12, 0.32), (0.28, 0.04), (0.70, 0.04), (0.88, 0.32), (0.70, 0.50), (0.28, 0.50), (0.12, 0.68), (0.28, 0.97), (0.70, 0.97), (0.88, 0.68), (0.70, 0.50))])
    add("9", [_poly((0.88, 0.62), (0.70, 0.97), (0.30, 0.97), (0.10, 0.80), (0.10, 0.64), (0.30, 0.50), (0.88, 0.64), (0.88, 0.20), (0.70, 0.04), (0.30, 0.04), (0.12, 0.18))])

    # Common punctuation and operators.  Narrow symbols carry a smaller
    # advance width, while the layout engine still applies standard spacing.
    add(".", [_line((0.50, 0.90), (0.50, 0.93))], 0.42)
    add(",", [_poly((0.53, 0.87), (0.50, 0.98), (0.40, 1.00))], 0.46)
    add(":", [_line((0.50, 0.30), (0.50, 0.33)), _line((0.50, 0.84), (0.50, 0.87))], 0.42)
    add(";", [_line((0.50, 0.30), (0.50, 0.33)), _poly((0.53, 0.82), (0.50, 0.98), (0.40, 1.00))], 0.46)
    add("!", [_line((0.50, 0.04), (0.50, 0.72)), _line((0.50, 0.92), (0.50, 0.95))], 0.42)
    add("?", [_poly((0.12, 0.20), (0.28, 0.04), (0.70, 0.04), (0.88, 0.20), (0.88, 0.36), (0.50, 0.62)), _line((0.50, 0.90), (0.50, 0.94))], 0.72)
    add("-", [_line((0.18, 0.50), (0.82, 0.50))], 0.68)
    add("_", [_line((0.05, 0.98), (0.95, 0.98))], 1.0)
    add("+", [_line((0.15, 0.50), (0.85, 0.50)), _line((0.50, 0.15), (0.50, 0.85))], 0.82)
    add("=", [_line((0.15, 0.38), (0.85, 0.38)), _line((0.15, 0.65), (0.85, 0.65))], 0.82)
    add("/", [_line((0.12, 1), (0.88, 0))], 0.72)
    add("\\", [_line((0.12, 0), (0.88, 1))], 0.72)
    add("|", [_line((0.50, 0), (0.50, 1))], 0.42)
    add("(", [_poly((0.68, 0.02), (0.42, 0.20), (0.30, 0.50), (0.42, 0.80), (0.68, 0.98))], 0.52)
    add(")", [_poly((0.32, 0.02), (0.58, 0.20), (0.70, 0.50), (0.58, 0.80), (0.32, 0.98))], 0.52)
    add("[", [_line((0.70, 0.02), (0.30, 0.02), (0.30, 0.98), (0.70, 0.98))], 0.56)
    add("]", [_line((0.30, 0.02), (0.70, 0.02), (0.70, 0.98), (0.30, 0.98))], 0.56)
    add("{", [_poly((0.68, 0.02), (0.48, 0.02), (0.38, 0.18), (0.38, 0.42), (0.22, 0.50), (0.38, 0.58), (0.38, 0.82), (0.48, 0.98), (0.68, 0.98))], 0.60)
    add("}", [_poly((0.32, 0.02), (0.52, 0.02), (0.62, 0.18), (0.62, 0.42), (0.78, 0.50), (0.62, 0.58), (0.62, 0.82), (0.52, 0.98), (0.32, 0.98))], 0.60)
    add("<", [_poly((0.78, 0.12), (0.20, 0.50), (0.78, 0.88))], 0.82)
    add(">", [_poly((0.22, 0.12), (0.80, 0.50), (0.22, 0.88))], 0.82)
    add("^", [_poly((0.12, 0.62), (0.50, 0.16), (0.88, 0.62))], 0.82)
    add("~", [_poly((0.10, 0.44), (0.28, 0.34), (0.50, 0.56), (0.72, 0.66), (0.90, 0.46))], 0.86)
    add("*", [_line((0.20, 0.25), (0.80, 0.75)), _line((0.80, 0.25), (0.20, 0.75)), _line((0.50, 0.12), (0.50, 0.88))], 0.78)
    add("'", [_line((0.50, 0.04), (0.50, 0.30))], 0.36)
    add('"', [_line((0.30, 0.04), (0.30, 0.30)), _line((0.70, 0.04), (0.70, 0.30))], 0.56)
    add("`", [_poly((0.58, 0.04), (0.42, 0.18), (0.34, 0.18))], 0.42)
    add("@", [_poly((0.70, 0.58), (0.70, 0.34), (0.55, 0.20), (0.30, 0.22), (0.14, 0.42), (0.14, 0.72), (0.32, 0.92), (0.68, 0.92), (0.86, 0.72), (0.86, 0.32), (0.74, 0.18)), _poly((0.70, 0.58), (0.52, 0.46), (0.34, 0.50), (0.30, 0.70), (0.46, 0.80), (0.62, 0.72), (0.70, 0.58))])
    add("#", [_line((0.30, 0.05), (0.20, 0.95)), _line((0.70, 0.05), (0.60, 0.95)), _line((0.10, 0.35), (0.90, 0.35)), _line((0.06, 0.68), (0.86, 0.68))], 0.86)
    add("$", [_poly((0.72, 0.16), (0.58, 0.04), (0.30, 0.04), (0.12, 0.18), (0.12, 0.38), (0.30, 0.50), (0.68, 0.50), (0.86, 0.64), (0.86, 0.84), (0.68, 0.97), (0.30, 0.97), (0.12, 0.84)), _line((0.52, 0), (0.52, 1))], 0.82)
    add("%", [_line((0.18, 0.20), (0.34, 0.36)), _poly((0.26, 0.05), (0.14, 0.18), (0.26, 0.31), (0.40, 0.18), (0.26, 0.05)), _poly((0.74, 0.69), (0.60, 0.82), (0.74, 0.96), (0.88, 0.82), (0.74, 0.69)), _line((0.82, 0.10), (0.18, 0.90))], 0.90)
    add("&", [_poly((0.80, 0.82), (0.62, 0.97), (0.28, 0.97), (0.10, 0.78), (0.10, 0.60), (0.30, 0.42), (0.70, 0.18), (0.72, 0.05), (0.52, 0.02), (0.30, 0.18), (0.30, 0.32), (0.70, 0.82))])

    # Space is represented explicitly so callers can use get_glyph(' ') and
    # retain a stable advance during layout.
    add(" ", [], 1.0)

    # Ensure every printable ASCII code has an entry.  A boxed fallback is
    # preferable to silently dropping an unknown punctuation mark.
    for code in range(32, 127):
        ch = chr(code)
        if ch not in s:
            add(ch, _box(0.15, 0.15, 0.85, 0.85), 0.86)
    return s


class AsciiFont:
    """Load and serve normalised single-line ASCII glyphs.

    Parameters
    ----------
    font_file:
        JSON file path.  If omitted, the package's ``data/ascii_single_line``
        file is tried and the built-in complete font is used as fallback.
    strict:
        If true, malformed/missing external files raise immediately.  The
        default is forgiving for ROS launch/demo use.
    """

    def __init__(self, font_file: Optional[str] = None, strict: bool = False):
        self.strict = bool(strict)
        package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
        if font_file and not os.path.isabs(os.fspath(font_file)):
            package_relative = os.path.join(package_root, os.fspath(font_file))
            self.font_file = package_relative if os.path.isfile(package_relative) else os.fspath(font_file)
        else:
            self.font_file = font_file or os.path.join(package_root, "data", "ascii_single_line.json")
        self._data: Dict[str, Dict[str, Any]] = {}
        self._cache: Dict[str, Glyph] = {}
        self.load_database()

    def load_database(self) -> Dict[str, Dict[str, Any]]:
        """Load JSON glyphs and merge missing entries from the built-in font."""

        builtin = _builtin_font_data()
        loaded: Dict[str, Any] = {}
        if self.font_file and os.path.isfile(self.font_file):
            try:
                with open(self.font_file, "r", encoding="utf-8") as handle:
                    raw = json.load(handle)
                if isinstance(raw, Mapping):
                    loaded = dict(raw)
                elif isinstance(raw, list):
                    # Accept a convenient list-of-records format as well as
                    # the canonical object keyed by character.
                    for record in raw:
                        if not isinstance(record, Mapping):
                            continue
                        key = record.get("char", record.get("character", record.get("symbol")))
                        if isinstance(key, str) and key:
                            loaded[key[0]] = record
                else:
                    raise ValueError("font JSON root must be an object or list")
            except Exception as exc:  # pragma: no cover - depends on user file
                if self.strict:
                    raise
                LOG.warning("Could not load ASCII font %s: %s; using built-in font", self.font_file, exc)
        elif self.strict:
            raise IOError("ASCII font file does not exist: {}".format(self.font_file))

        # External data wins, but fill gaps so all printable characters remain
        # available even if an abbreviated JSON was supplied.
        self._data = dict(builtin)
        self._data.update(loaded)
        self._cache.clear()
        return self._data

    @staticmethod
    def _entry_to_glyph(ch: str, entry: Any) -> Glyph:
        width = 1.0
        height = 1.0
        strokes_raw: Any = entry
        metadata: Dict[str, Any] = {}
        if isinstance(entry, Mapping):
            strokes_raw = entry.get("strokes", entry.get("points", []))
            width = float(entry.get("width", entry.get("advance", 1.0)))
            height = float(entry.get("height", 1.0))
            metadata = {k: v for k, v in entry.items() if k not in ("strokes", "points", "width", "advance", "height")}
        if strokes_raw is None:
            strokes_raw = []
        # A single polyline may be provided directly as [[u,v], ...].
        if isinstance(strokes_raw, Sequence) and strokes_raw:
            first = strokes_raw[0]
            if isinstance(first, Sequence) and len(first) >= 2 and isinstance(first[0], (int, float)):
                strokes_raw = [strokes_raw]
        strokes: List[Stroke] = []
        for raw_stroke in strokes_raw:
            try:
                points = [Point2D(float(p[0]), float(p[1])) for p in raw_stroke if isinstance(p, Sequence) and len(p) >= 2]
            except (TypeError, ValueError):
                points = []
            if points:
                strokes.append(Stroke(points))
        return Glyph(ch, strokes, width, height, metadata)

    def has_glyph(self, char: str) -> bool:
        return bool(char) and _FULLWIDTH_ALIASES.get(char[0], char[0]) in self._data

    def get_glyph(self, char: str) -> Glyph:
        """Return a deep-copy glyph for ``char``.

        Newline and other control characters are blank advances.  Non-ASCII
        input is represented by a boxed fallback glyph, allowing mixed text to
        be rendered without a hard failure while still making unsupported data
        visually obvious.
        """

        if not char:
            return Glyph("", [], 1.0, 1.0)
        ch = char[0]
        source_ch = ch
        ch = _FULLWIDTH_ALIASES.get(ch, ch)
        if ch in self._cache:
            glyph = self._cache[ch].copy()
            if source_ch != ch:
                glyph.symbol = source_ch
                glyph.metadata["alias_for"] = ch
            return glyph
        entry = self._data.get(ch)
        if entry is None:
            # Keep the return type stable for callers.  A blank control glyph
            # is preferable to drawing a misleading box for line breaks.
            if ch in "\r\n\t":
                glyph = Glyph(ch, [], 1.0, 1.0, {"control": True})
            else:
                glyph = self._entry_to_glyph(ch, _builtin_font_data().get("?"))
                glyph.metadata["fallback_for"] = ch
            self._cache[ch] = glyph
            result = glyph.copy()
            if source_ch != ch:
                result.symbol = source_ch
                result.metadata["alias_for"] = ch
            return result
        glyph = self._entry_to_glyph(ch, entry)
        self._cache[ch] = glyph
        result = glyph.copy()
        if source_ch != ch:
            result.symbol = source_ch
            result.metadata["alias_for"] = ch
        return result

    def glyphs_for_text(self, text: str) -> List[Glyph]:
        return [self.get_glyph(ch) for ch in str(text)]

    @property
    def supported_characters(self) -> str:
        return "".join(sorted(ch for ch in self._data if len(ch) == 1 and ch in string.printable))


__all__ = ["AsciiFont"]
