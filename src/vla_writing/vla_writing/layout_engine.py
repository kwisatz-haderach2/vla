"""Paper-aware text layout with automatic line wrapping.

``LayoutEngine`` is the common stage after all parsers.  It knows nothing
about ROS or robot poses: it places local glyph strokes in a paper rectangle,
using the project's logical coordinates (``u`` right, ``v`` down).  English,
Hanzi and image glyphs therefore share exactly the same downstream motion
pipeline.
"""

from __future__ import annotations

import logging
import math
import re
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

from .data_types import Glyph, Point2D, Stroke, clone_strokes


LOG = logging.getLogger(__name__)


class LayoutEngine:
    """Lay out mixed ASCII/Hanzi glyphs on a bounded paper rectangle.

    All dimensions are metres.  The defaults match the technical plan: A3-ish
    paper, 25x40 mm English characters, 40 mm Chinese characters, 0.5-char
    letter/Hanzi spacing and 1-char word spacing.
    """

    def __init__(
        self,
        paper_width: float = 0.420,
        paper_height: float = 0.297,
        margin_left: float = 0.020,
        margin_top: float = 0.020,
        margin_right: float = 0.020,
        margin_bottom: float = 0.020,
        line_spacing_ratio: float = 0.5,
        english_char_width: float = 0.025,
        english_char_height: float = 0.040,
        letter_spacing_ratio: float = 0.5,
        word_spacing_ratio: float = 1.0,
        chinese_char_width: float = 0.040,
        chinese_char_height: float = 0.040,
        chinese_spacing_ratio: float = 0.5,
        image_width: Optional[float] = None,
        image_height: Optional[float] = None,
        wrap_words: bool = True,
        config: Optional[Mapping[str, Any]] = None,
    ):
        # Permit the concise ``LayoutEngine(config_dict)`` form in addition
        # to the explicit ``LayoutEngine(config=config_dict)`` API.
        if isinstance(paper_width, Mapping) and config is None:
            config = paper_width
            paper_width = 0.420
        if config is not None:
            values = self._flatten_config(config)
            paper = config.get("paper", {}) if isinstance(config, Mapping) else {}
            layout = config.get("layout", {}) if isinstance(config, Mapping) else {}
            english = config.get("english", {}) if isinstance(config, Mapping) else {}
            chinese = config.get("chinese", {}) if isinstance(config, Mapping) else {}
            paper_width = values.get("paper_width", paper.get("width", paper_width))
            paper_height = values.get("paper_height", paper.get("height", paper_height))
            margin_left = layout.get("margin_left", margin_left)
            margin_top = layout.get("margin_top", margin_top)
            margin_right = layout.get("margin_right", margin_right)
            margin_bottom = layout.get("margin_bottom", margin_bottom)
            line_spacing_ratio = layout.get("line_spacing_ratio", line_spacing_ratio)
            wrap_words = layout.get("wrap_words", wrap_words)
            english_char_width = english.get("char_width", english_char_width)
            english_char_height = english.get("char_height", english_char_height)
            letter_spacing_ratio = english.get("letter_spacing_ratio", letter_spacing_ratio)
            word_spacing_ratio = english.get("word_spacing_ratio", word_spacing_ratio)
            chinese_char_width = chinese.get("char_width", chinese_char_width)
            chinese_char_height = chinese.get("char_height", chinese_char_height)
            chinese_spacing_ratio = chinese.get("spacing_ratio", chinese_spacing_ratio)
        self.paper_width = float(paper_width)
        self.paper_height = float(paper_height)
        self.margin_left = max(0.0, float(margin_left))
        self.margin_top = max(0.0, float(margin_top))
        self.margin_right = max(0.0, float(margin_right))
        self.margin_bottom = max(0.0, float(margin_bottom))
        self.line_spacing_ratio = max(0.0, float(line_spacing_ratio))
        self.english_char_width = float(english_char_width)
        self.english_char_height = float(english_char_height)
        self.letter_spacing_ratio = max(0.0, float(letter_spacing_ratio))
        self.word_spacing_ratio = max(0.0, float(word_spacing_ratio))
        self.chinese_char_width = float(chinese_char_width)
        self.chinese_char_height = float(chinese_char_height)
        self.chinese_spacing_ratio = max(0.0, float(chinese_spacing_ratio))
        self.image_width = image_width
        self.image_height = image_height
        self.wrap_words = bool(wrap_words)
        self.last_line_count = 0
        self.last_overflow = False

    @staticmethod
    def _flatten_config(config: Mapping[str, Any]) -> Dict[str, Any]:
        paper = config.get("paper", {})
        values: Dict[str, Any] = {}
        if isinstance(paper, Mapping):
            if paper.get("width") is not None:
                values["paper_width"] = paper.get("width")
            if paper.get("height") is not None:
                values["paper_height"] = paper.get("height")
        return values

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> "LayoutEngine":
        return cls(config=config)

    @staticmethod
    def is_hanzi(symbol: str) -> bool:
        if not symbol:
            return False
        code = ord(symbol[0])
        return (
            0x3400 <= code <= 0x4DBF
            or 0x4E00 <= code <= 0x9FFF
            or 0xF900 <= code <= 0xFAFF
            or 0x20000 <= code <= 0x2FA1F
        )

    @classmethod
    def glyph_kind(cls, glyph: Glyph) -> str:
        symbol = glyph.symbol or ""
        metadata = glyph.metadata or {}
        if metadata.get("image") or metadata.get("source") == "opencv_contour":
            return "image"
        if metadata.get("hanzi") or cls.is_hanzi(symbol):
            return "hanzi"
        if symbol in ("\n", "\r"):
            return "newline"
        if symbol in (" ", "\t") or metadata.get("control"):
            return "space"
        return "ascii"

    def _dimensions(self, glyph: Glyph, kind: str) -> Tuple[float, float]:
        if kind == "hanzi":
            return self.chinese_char_width * max(0.01, glyph.width), self.chinese_char_height * max(0.01, glyph.height)
        if kind == "image":
            width = self.image_width if self.image_width is not None else self.paper_width - self.margin_left - self.margin_right
            height = self.image_height if self.image_height is not None else self.paper_height - self.margin_top - self.margin_bottom
            return float(width) * max(0.01, glyph.width), float(height) * max(0.01, glyph.height)
        return self.english_char_width * max(0.01, glyph.width), self.english_char_height * max(0.01, glyph.height)

    def _spacing(self, previous_kind: Optional[str], next_kind: str, explicit_word: bool = False) -> float:
        if explicit_word:
            return self.english_char_width * self.word_spacing_ratio
        if previous_kind == "hanzi" and next_kind == "hanzi":
            return self.chinese_char_width * self.chinese_spacing_ratio
        if previous_kind == "image" or next_kind == "image":
            return self.english_char_width * self.letter_spacing_ratio
        if previous_kind is None:
            return 0.0
        return self.english_char_width * self.letter_spacing_ratio

    def _place_glyph(self, glyph: Glyph, kind: str, cursor_u: float, cursor_v: float, line_index: int) -> List[Stroke]:
        width, height = self._dimensions(glyph, kind)
        placed: List[Stroke] = []
        for stroke in glyph.strokes:
            if len(stroke.points) == 0:
                continue
            points = [Point2D(cursor_u + p.u * width, cursor_v + p.v * height) for p in stroke.points]
            metadata = dict(stroke.metadata)
            metadata.update({"symbol": glyph.symbol, "kind": kind, "line": line_index})
            placed.append(Stroke(points, stroke.closed, metadata))
        return placed

    @staticmethod
    def _tokenize(glyphs: Sequence[Glyph]) -> List[Tuple[str, List[Glyph]]]:
        """Group ASCII words while keeping Hanzi and whitespace as units."""

        tokens: List[Tuple[str, List[Glyph]]] = []
        current: List[Glyph] = []
        for glyph in glyphs:
            kind = LayoutEngine.glyph_kind(glyph)
            if kind == "ascii":
                current.append(glyph)
                continue
            if current:
                tokens.append(("ascii_word", current))
                current = []
            tokens.append((kind, [glyph]))
        if current:
            tokens.append(("ascii_word", current))
        return tokens

    def _word_width(self, glyphs: Sequence[Glyph]) -> float:
        total = 0.0
        for idx, glyph in enumerate(glyphs):
            width, _ = self._dimensions(glyph, "ascii")
            total += width
            if idx:
                total += self.english_char_width * self.letter_spacing_ratio
        return total

    def _place_word(
        self,
        glyphs: Sequence[Glyph],
        cursor_u: float,
        cursor_v: float,
        line_index: int,
    ) -> Tuple[List[Stroke], float, float, Optional[str]]:
        strokes: List[Stroke] = []
        previous: Optional[str] = None
        max_height = 0.0
        for glyph in glyphs:
            kind = "ascii"
            width, height = self._dimensions(glyph, kind)
            if previous is not None:
                cursor_u += self._spacing(previous, kind)
            strokes.extend(self._place_glyph(glyph, kind, cursor_u, cursor_v, line_index))
            cursor_u += width
            max_height = max(max_height, height)
            previous = kind
        return strokes, cursor_u, max_height, previous

    def layout_text(
        self,
        glyphs_or_text: Union[str, Sequence[Glyph]],
        ascii_font: Optional[Any] = None,
        hanzi_parser: Optional[Any] = None,
    ) -> List[Stroke]:
        """Lay out a string or a pre-parsed glyph list.

        Passing a string is a convenience: ``ascii_font`` defaults to
        :class:`AsciiFont` and ``hanzi_parser`` to :class:`HanziParser`.  For
        maximum control (and fast unit tests), pass a list of :class:`Glyph`
        objects directly.
        """

        if isinstance(glyphs_or_text, str):
            if ascii_font is None:
                from .ascii_font import AsciiFont

                ascii_font = AsciiFont()
            if hanzi_parser is None:
                from .hanzi_parser import HanziParser

                hanzi_parser = HanziParser(warn_missing=False)
            glyphs: List[Glyph] = []
            for char in glyphs_or_text:
                if self.is_hanzi(char):
                    glyphs.append(hanzi_parser.get_glyph(char))
                else:
                    glyphs.append(ascii_font.get_glyph(char))
        else:
            glyphs = []
            for item in glyphs_or_text:
                if isinstance(item, Glyph):
                    glyphs.append(item)
                elif isinstance(item, Stroke):
                    glyphs.append(Glyph("<stroke>", [item]))
                else:
                    glyphs.append(Glyph("?", [Stroke(item)]))

        left = self.margin_left
        right = self.paper_width - self.margin_right
        top = self.margin_top
        bottom = self.paper_height - self.margin_bottom
        available_width = max(0.0, right - left)
        line_step = max(self.english_char_height, self.chinese_char_height) * (1.0 + self.line_spacing_ratio)
        cursor_u = left
        cursor_v = top
        line_index = 0
        line_has_content = False
        previous_kind: Optional[str] = None
        pending_word_space = False
        output: List[Stroke] = []
        self.last_overflow = False

        def newline() -> None:
            nonlocal cursor_u, cursor_v, line_index, line_has_content, previous_kind, pending_word_space
            cursor_u = left
            cursor_v += line_step
            line_index += 1
            line_has_content = False
            previous_kind = None
            pending_word_space = False

        tokens = self._tokenize(glyphs)
        for token_kind, token_glyphs in tokens:
            if token_kind == "newline":
                newline()
                continue
            if token_kind == "space":
                if line_has_content:
                    pending_word_space = True
                continue
            if token_kind == "ascii_word":
                token_width = self._word_width(token_glyphs)
                # A single unbroken token (for example a long URL) can be
                # wider than the sheet.  Keep normal words intact, but fall
                # back to character-level wrapping for this corner case so a
                # pathological input cannot push every subsequent point off
                # the paper.
                if self.wrap_words and token_width > available_width + 1e-9 and len(token_glyphs) > 1:
                    for glyph in token_glyphs:
                        width, height = self._dimensions(glyph, "ascii")
                        leading = self._spacing(previous_kind, "ascii") if line_has_content else 0.0
                        if line_has_content and cursor_u + leading + width > right + 1e-9:
                            newline()
                            leading = 0.0
                        if cursor_v + height > bottom + 1e-9:
                            self.last_overflow = True
                            LOG.warning("Text exceeds paper height; remaining glyphs skipped")
                            break
                        cursor_u += leading
                        output.extend(self._place_glyph(glyph, "ascii", cursor_u, cursor_v, line_index))
                        cursor_u += width
                        line_has_content = True
                        previous_kind = "ascii"
                    pending_word_space = False
                    if self.last_overflow:
                        break
                    continue
                leading = self._spacing(previous_kind, "ascii", explicit_word=pending_word_space) if line_has_content else 0.0
                if self.wrap_words and line_has_content and cursor_u + leading + token_width > right + 1e-9:
                    newline()
                    leading = 0.0
                if cursor_v + self.english_char_height > bottom + 1e-9:
                    self.last_overflow = True
                    LOG.warning("Text exceeds paper height; remaining glyphs skipped")
                    break
                cursor_u += leading
                placed, cursor_u, _, previous_kind = self._place_word(token_glyphs, cursor_u, cursor_v, line_index)
                output.extend(placed)
                line_has_content = True
                pending_word_space = False
                continue
            # A Hanzi or image is an independent token.  Missing Hanzi glyphs
            # still receive an advance, making omission visible in spacing.
            glyph = token_glyphs[0]
            kind = token_kind
            width, height = self._dimensions(glyph, kind)
            leading = self._spacing(previous_kind, kind, explicit_word=pending_word_space) if line_has_content else 0.0
            if line_has_content and cursor_u + leading + width > right + 1e-9:
                newline()
                leading = 0.0
            if cursor_v + height > bottom + 1e-9:
                self.last_overflow = True
                LOG.warning("Text exceeds paper height; remaining glyphs skipped")
                break
            cursor_u += leading
            output.extend(self._place_glyph(glyph, kind, cursor_u, cursor_v, line_index))
            cursor_u += width
            line_has_content = True
            previous_kind = kind
            pending_word_space = False

        self.last_line_count = line_index + 1 if glyphs else 0
        return output

    # Backwards-compatible aliases used by early demo scripts.
    layout_glyphs = layout_text
    layout_string = layout_text

    def layout_strokes(
        self,
        strokes: Iterable[Stroke],
        width: Optional[float] = None,
        height: Optional[float] = None,
        origin_u: Optional[float] = None,
        origin_v: Optional[float] = None,
        padding: float = 0.02,
    ) -> List[Stroke]:
        """Fit image strokes into the available paper area.

        Input points may already be normalised or may be arbitrary pixel
        coordinates; both are handled by computing a bounding box first.
        """

        source = [s.copy() for s in strokes if len(s.points) >= 2]
        if not source:
            return []
        points = [p for s in source for p in s.points]
        min_u, max_u = min(p.u for p in points), max(p.u for p in points)
        min_v, max_v = min(p.v for p in points), max(p.v for p in points)
        span_u, span_v = max(max_u - min_u, 1e-12), max(max_v - min_v, 1e-12)
        box_w = float(width if width is not None else self.paper_width - self.margin_left - self.margin_right)
        box_h = float(height if height is not None else self.paper_height - self.margin_top - self.margin_bottom)
        origin_u = self.margin_left if origin_u is None else float(origin_u)
        origin_v = self.margin_top if origin_v is None else float(origin_v)
        pad = max(0.0, min(0.49, float(padding)))
        scale = min(box_w * (1 - 2 * pad) / span_u, box_h * (1 - 2 * pad) / span_v)
        used_w, used_h = span_u * scale, span_v * scale
        offset_u = origin_u + pad * box_w + (box_w * (1 - 2 * pad) - used_w) * 0.5
        offset_v = origin_v + pad * box_h + (box_h * (1 - 2 * pad) - used_h) * 0.5
        return [
            Stroke(
                [Point2D(offset_u + (p.u - min_u) * scale, offset_v + (p.v - min_v) * scale) for p in s.points],
                s.closed,
                dict(s.metadata),
            )
            for s in source
        ]


__all__ = ["LayoutEngine"]
