#!/usr/bin/env python3
"""Publish RViz markers matching the static Gazebo writing scene.

Gazebo SDF ``<include>`` models are not part of ``robot_description`` and
therefore do not appear in RViz's RobotModel display.  This node mirrors the
authoritative dimensions/poses from writing.world and the three SDF models so
RViz can show the same tabletop, paper, camera and writing workspace.
"""

from __future__ import print_function

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


FRAME = "world"


def _box(marker_id, ns, x, y, z, sx, sy, sz, color):
    marker = Marker()
    marker.header.frame_id = FRAME
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.w = 1.0
    marker.scale.x = sx
    marker.scale.y = sy
    marker.scale.z = sz
    marker.color = color
    marker.lifetime = rospy.Duration(0)
    return marker


def _text(marker_id, text, x, y, z, color):
    marker = Marker()
    marker.header.frame_id = FRAME
    marker.ns = "labels"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.035
    marker.color = color
    marker.text = text
    marker.lifetime = rospy.Duration(0)
    return marker


def _arrow(marker_id, ns, start, end, color):
    marker = Marker()
    marker.header.frame_id = FRAME
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.004
    marker.scale.y = 0.008
    marker.scale.z = 0.012
    marker.color = color
    marker.points = [Point(*start), Point(*end)]
    marker.lifetime = rospy.Duration(0)
    return marker


def build_markers():
    markers = []
    brown = ColorRGBA(0.48, 0.27, 0.12, 1.0)
    dark_brown = ColorRGBA(0.36, 0.18, 0.06, 1.0)
    shelf_brown = ColorRGBA(0.28, 0.13, 0.04, 0.9)
    white = ColorRGBA(0.98, 0.98, 0.98, 0.95)
    border = ColorRGBA(0.25, 0.25, 0.25, 0.85)
    camera = ColorRGBA(0.08, 0.10, 0.14, 1.0)
    label = ColorRGBA(0.95, 0.95, 0.95, 1.0)

    # writing_table include pose is (0.39, 0.0515, 0).  Link poses and sizes
    # below are copied from models/writing_table/model.sdf.
    tx, ty = 0.39, 0.0515
    markers.append(_box(0, "table", tx, ty, 0.487, 0.90, 0.70, 0.06, brown))
    leg_id = 1
    for lx in (-0.37, 0.37):
        for ly in (-0.27, 0.27):
            markers.append(_box(leg_id, "table", tx + lx, ty + ly, 0.23,
                                0.07, 0.07, 0.46, dark_brown))
            leg_id += 1
    markers.append(_box(5, "table", tx, ty, 0.12, 0.70, 0.48, 0.025, shelf_brown))

    # writing_paper include origin is (0.18, 0.20, 0).  The sheet centre is
    # therefore (0.39, 0.0515) and its visible top is z=0.521.
    markers.append(_box(0, "paper", tx, ty, 0.519, 0.42, 0.297, 0.004, white))
    markers.append(_box(1, "paper", tx, ty, 0.5215, 0.425, 0.302, 0.001, border))

    # paper_camera include origin is the sheet centre; its link is at z=1.35.
    markers.append(_box(0, "camera", tx, ty, 1.35, 0.08, 0.08, 0.06, camera))

    markers.append(_text(0, "writing_table", tx - 0.40, ty - 0.36, 0.57, label))
    markers.append(_text(1, "paper", tx + 0.24, ty + 0.17, 0.55, label))
    markers.append(_text(2, "paper_camera", tx + 0.06, ty + 0.06, 1.42, label))

    # Paper-frame axes: +X points right across the sheet and +Y points toward
    # its top edge, matching the planner's paper_frame convention.
    markers.append(_arrow(0, "paper_axes", (0.39, 0.0515, 0.523),
                          (0.49, 0.0515, 0.523), ColorRGBA(0.9, 0.1, 0.1, 1.0)))
    markers.append(_arrow(1, "paper_axes", (0.39, 0.0515, 0.523),
                          (0.39, 0.1515, 0.523), ColorRGBA(0.1, 0.9, 0.1, 1.0)))
    markers.append(_arrow(2, "paper_axes", (0.39, 0.0515, 0.523),
                          (0.39, 0.0515, 0.623), ColorRGBA(0.1, 0.4, 1.0, 1.0)))
    return MarkerArray(markers=markers)


def main():
    rospy.init_node("scene_markers")
    topic = rospy.get_param("~topic", "/vla/scene_markers")
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=1, latch=True)
    message = build_markers()
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        publisher.publish(message)
        rate.sleep()


if __name__ == "__main__":
    main()
