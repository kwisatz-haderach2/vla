#!/usr/bin/env python3
"""OpenCV top-down paper snapshot built from the *actual* pen TF."""

from __future__ import print_function

import os
import threading

import cv2
import numpy as np
import rospy
import yaml

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String


def _load_config():
    default = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "config", "writing.yaml"))
    path = rospy.get_param("~config", default)
    try:
        with open(os.path.expanduser(path), "r") as handle:
            return yaml.safe_load(handle) or {}
    except Exception as exc:
        rospy.logwarn("paper_view config unavailable: %s", exc)
        return {}


class PaperView(object):
    def __init__(self):
        cfg = _load_config()
        paper = cfg.get("paper", {})
        opencv = cfg.get("opencv", {})
        self.paper_width = float(paper.get("width", 0.42))
        self.paper_height = float(paper.get("height", 0.297))
        self.width = int(rospy.get_param("~width", opencv.get("width", 840)))
        self.height = int(rospy.get_param("~height", opencv.get("height", 594)))
        self.thickness = int(rospy.get_param("~thickness", opencv.get("thickness", 3)))
        self.window_name = str(rospy.get_param("~window_name", opencv.get("window_name", "paper_snapshot")))
        self.show_window = bool(rospy.get_param("~show_window", opencv.get("show_window", True)))
        self.save_path = os.path.expanduser(str(rospy.get_param("~save_path", opencv.get("save_path", "/tmp/vla_paper_snapshot.png"))))
        if not os.environ.get("DISPLAY"):
            self.show_window = False
        self.canvas = np.full((self.height, self.width, 3), 255, dtype=np.uint8)
        self.lock = threading.RLock()
        self.contact = False
        self.previous_pixel = None
        self.last_status = "idle"
        self.pose_sub = rospy.Subscriber("/vla/pen_pose", PoseStamped, self._pose_cb, queue_size=20)
        self.contact_sub = rospy.Subscriber("/vla/pen_contact", Bool, self._contact_cb, queue_size=20)
        self.status_sub = rospy.Subscriber("/vla/status", String, self._status_cb, queue_size=5)
        rospy.on_shutdown(self.save)

    def _contact_cb(self, message):
        with self.lock:
            self.contact = bool(message.data)
            if not self.contact:
                # Do not connect two separate strokes through an air move.
                self.previous_pixel = None

    def _status_cb(self, message):
        self.last_status = message.data

    def _pose_cb(self, message):
        with self.lock:
            x = float(message.pose.position.x)
            y = float(message.pose.position.y)
            px = int(round(x / self.paper_width * (self.width - 1)))
            py = int(round((-y) / self.paper_height * (self.height - 1)))
            if not (0 <= px < self.width and 0 <= py < self.height):
                self.previous_pixel = None
                return
            current = (px, py)
            if self.contact and self.previous_pixel is not None:
                cv2.line(self.canvas, self.previous_pixel, current, (15, 15, 15), self.thickness, cv2.LINE_AA)
            self.previous_pixel = current if self.contact else None

    def _annotated(self):
        with self.lock:
            image = self.canvas.copy()
            cv2.rectangle(image, (0, 0), (self.width - 1, self.height - 1), (190, 190, 190), 1)
            cv2.putText(image, "VLA writing | " + str(self.last_status), (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 80), 1, cv2.LINE_AA)
            return image

    def save(self):
        try:
            directory = os.path.dirname(self.save_path)
            if directory and not os.path.isdir(directory):
                os.makedirs(directory)
            cv2.imwrite(self.save_path, self._annotated())
            rospy.loginfo("paper_snapshot saved to %s", self.save_path)
        except Exception as exc:
            rospy.logwarn("Could not save paper snapshot: %s", exc)

    def spin(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            if self.show_window:
                try:
                    cv2.imshow(self.window_name, self._annotated())
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        rospy.signal_shutdown("paper_view quit")
                except Exception as exc:
                    rospy.logwarn_throttle(5.0, "OpenCV window disabled: %s", exc)
                    self.show_window = False
            rate.sleep()


def main():
    rospy.init_node("paper_view")
    PaperView().spin()


if __name__ == "__main__":
    main()
