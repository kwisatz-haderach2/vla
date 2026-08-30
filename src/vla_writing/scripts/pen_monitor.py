#!/usr/bin/env python3
"""Publish the actual UR5e pen-tip pose and contact state in ROS1."""

from __future__ import print_function

import os

import rospy
import yaml
import tf2_ros

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


def _config():
    default = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, "config", "writing.yaml"))
    path = rospy.get_param("~config", default)
    try:
        with open(os.path.expanduser(path), "r") as f:
            return yaml.safe_load(f) or {}
    except Exception as exc:
        rospy.logwarn("Could not load writing config %s: %s", path, exc)
        return {}


class PenMonitor(object):
    def __init__(self):
        cfg = _config()
        paper = cfg.get("paper", {})
        motion = cfg.get("motion", {})
        ink = cfg.get("ink", {})
        self.paper_frame = rospy.get_param("~paper_frame", paper.get("frame", "paper_frame"))
        self.pen_frame = rospy.get_param("~pen_frame", "pen_tip")
        self.width = float(paper.get("width", 0.42))
        self.height = float(paper.get("height", 0.297))
        self.write_z = float(motion.get("write_z", 0.0015))
        self.threshold = float(ink.get("contact_threshold", 0.003))
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pose_pub = rospy.Publisher("/vla/pen_pose", PoseStamped, queue_size=10)
        self.contact_pub = rospy.Publisher("/vla/pen_contact", Bool, queue_size=10)

    def spin(self):
        rate = rospy.Rate(max(1.0, self.rate_hz))
        while not rospy.is_shutdown():
            contact = False
            try:
                transform = self.buffer.lookup_transform(
                    self.paper_frame, self.pen_frame, rospy.Time(0), rospy.Duration(0.05))
                t = transform.transform.translation
                q = transform.transform.rotation
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = self.paper_frame
                pose.pose.position.x = t.x
                pose.pose.position.y = t.y
                pose.pose.position.z = t.z
                pose.pose.orientation = q
                self.pose_pub.publish(pose)
                contact = (
                    0.0 <= t.x <= self.width and
                    -self.height <= t.y <= 0.0 and
                    abs(t.z - self.write_z) <= self.threshold
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            except Exception as exc:
                rospy.logdebug("pen TF unavailable: %s", exc)
            self.contact_pub.publish(Bool(data=contact))
            rate.sleep()


def main():
    rospy.init_node("pen_monitor")
    PenMonitor().spin()


if __name__ == "__main__":
    main()
