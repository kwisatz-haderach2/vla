#!/usr/bin/env python3
"""ROS1 service node for English, Hanzi and image writing.

The node intentionally contains very little geometry logic.  It selects a
parser, asks ``LayoutEngine`` for paper-space strokes, then sends the same
``TrajectoryBuilder`` output to ``MoveItExecutor`` for every input type.
"""

from __future__ import print_function

import copy
import logging
import os
import threading
import traceback

import rospy
import yaml

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from vla_writing.ascii_font import AsciiFont
from vla_writing.hanzi_parser import HanziParser
from vla_writing.image_parser import ImageParser
from vla_writing.layout_engine import LayoutEngine
from vla_writing.moveit_executor import MoveItExecutor
from vla_writing.trajectory_builder import TrajectoryBuilder
from vla_writing.srv import DrawImage, DrawImageResponse, WriteText, WriteTextResponse


LOG = logging.getLogger("vla_writing.writing_node")


def _package_path():
    try:
        import rospkg
        return rospkg.RosPack().get_path("vla_writing")
    except Exception:
        return os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))


def _resolve(package_path, value):
    if value is None:
        return value
    value = os.path.expanduser(os.path.expandvars(str(value)))
    if os.path.isabs(value):
        return value
    return os.path.join(package_path, value)


class WritingNode(object):
    def __init__(self):
        self.package_path = _package_path()
        config_arg = rospy.get_param("~config", os.path.join(self.package_path, "config", "writing.yaml"))
        self.config_path = _resolve(self.package_path, config_arg)
        with open(self.config_path, "r") as handle:
            self.config = yaml.safe_load(handle) or {}

        self.status_pub = rospy.Publisher("/vla/status", String, queue_size=10, latch=True)
        self.lock = threading.RLock()
        self.busy = False

        english_cfg = self.config.get("english", {})
        chinese_cfg = self.config.get("chinese", {})
        image_cfg = self.config.get("image", {})
        self.ascii_font = AsciiFont(_resolve(self.package_path, english_cfg.get("font_file")))
        self.hanzi_parser = HanziParser(_resolve(self.package_path, chinese_cfg.get("graphics_file")))
        self.image_parser = ImageParser(
            min_contour_length=float(image_cfg.get("min_contour_length", 30.0)),
            simplify_epsilon_ratio=float(image_cfg.get("simplify_epsilon_ratio", 0.003)),
            min_contour_area=float(image_cfg.get("min_contour_area", 4.0)),
            resample_spacing=float(self.config.get("motion", {}).get("point_spacing", 0.002)),
        )
        self.layout = LayoutEngine.from_config(self.config)

        motion = self.config.get("motion", {})
        paper = self.config.get("paper", {})
        self.builder = TrajectoryBuilder(
            write_z=float(motion.get("write_z", 0.0015)),
            lift_z=float(motion.get("lift_z", 0.030)),
            point_spacing=float(motion.get("point_spacing", 0.002)),
            orientation=motion.get("orientation", [0.0, 0.0, 0.0, 1.0]),
            paper_frame=str(paper.get("frame", "paper_frame")),
            base_frame=str(self.config.get("moveit", {}).get("base_frame", "base_link")),
        )
        dry_run = bool(rospy.get_param("~dry_run", False))
        self.executor = MoveItExecutor(self.config, self._publish_status, dry_run=dry_run)
        self.dry_run = dry_run

        self.write_text_srv = rospy.Service("/vla/write_text", WriteText, self._write_text)
        self.draw_image_srv = rospy.Service("/vla/draw_image", DrawImage, self._draw_image)
        self._publish_status("idle")
        rospy.loginfo("vla_writing ready: /vla/write_text and /vla/draw_image")

    def _publish_status(self, value):
        text = str(value)
        try:
            self.status_pub.publish(String(data=text))
        except Exception:
            pass

    def _glyphs_for_text(self, text):
        glyphs = []
        for char in str(text):
            if self.hanzi_parser.is_hanzi(char):
                glyphs.append(self.hanzi_parser.get_glyph(char))
            else:
                glyphs.append(self.ascii_font.get_glyph(char))
        return glyphs

    def _prepare_executor(self):
        if not self.executor.initialize():
            raise RuntimeError("MoveIt is not available; start demo.launch or use ~dry_run:=true")
        if not self.executor.move_to_ready_pose():
            raise RuntimeError("UR5e could not reach the ready pose")
        # Keep the orientation that the official UR5e SRDF's ``up`` state
        # actually produced.  This is more reliable than guessing a quaternion
        # when a site uses a different tool0 convention.
        self.builder.orientation = self.executor.current_orientation()

    def _execute_strokes(self, strokes):
        if not strokes:
            return False, "No drawable strokes were produced"
        self._publish_status("planning {} strokes".format(len(strokes)))
        self._prepare_executor()
        ok = self.executor.plan_with_fallback(strokes, self.builder)
        return bool(ok), ("completed" if ok else "Cartesian path planning/execution failed")

    def _begin(self):
        if not self.lock.acquire(False):
            return False
        if self.busy:
            self.lock.release()
            return False
        self.busy = True
        return True

    def _end(self):
        self.busy = False
        self.lock.release()

    def _write_text(self, request):
        if not self._begin():
            return WriteTextResponse(False, "A writing request is already running")
        try:
            text = request.text or ""
            if not text.strip():
                return WriteTextResponse(False, "text is empty")
            self._publish_status("parsing text")
            glyphs = self._glyphs_for_text(text)
            strokes = self.layout.layout_text(glyphs)
            ok, message = self._execute_strokes(strokes)
            self._publish_status("idle" if ok else "error: " + message)
            return WriteTextResponse(ok, message)
        except Exception as exc:
            LOG.error("write_text failed: %s\n%s", exc, traceback.format_exc())
            self._publish_status("error: {}".format(exc))
            return WriteTextResponse(False, str(exc))
        finally:
            self._end()

    def _draw_image(self, request):
        if not self._begin():
            return DrawImageResponse(False, "A writing request is already running")
        try:
            image_path = _resolve(self.package_path, request.image_path)
            if not image_path or not os.path.isfile(image_path):
                return DrawImageResponse(False, "Image does not exist: {}".format(image_path))
            self._publish_status("extracting OpenCV contours")
            raw_strokes = self.image_parser.generate_strokes(image_path, normalize=True)
            strokes = self.layout.layout_strokes(raw_strokes)
            ok, message = self._execute_strokes(strokes)
            self._publish_status("idle" if ok else "error: " + message)
            return DrawImageResponse(ok, message)
        except Exception as exc:
            LOG.error("draw_image failed: %s\n%s", exc, traceback.format_exc())
            self._publish_status("error: {}".format(exc))
            return DrawImageResponse(False, str(exc))
        finally:
            self._end()


def main():
    rospy.init_node("writing_node")
    WritingNode()
    rospy.spin()


if __name__ == "__main__":
    main()
