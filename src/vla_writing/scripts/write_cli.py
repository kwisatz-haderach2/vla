#!/usr/bin/env python3
"""Small, video-friendly command line client for the writing services."""

from __future__ import print_function

import argparse
import os
import sys

import rospy

from vla_writing.srv import DrawImage, WriteText


def main():
    parser = argparse.ArgumentParser(description="UR5e writing demo client")
    sub = parser.add_subparsers(dest="command")
    text = sub.add_parser("text", help="write a mixed English/Hanzi string")
    text.add_argument("value", nargs="+", help="text to write")
    image = sub.add_parser("image", help="extract and draw an OpenCV line-art image")
    image.add_argument("path", help="image path")
    args = parser.parse_args()
    if args.command not in ("text", "image"):
        parser.print_help()
        return 2

    rospy.init_node("write_cli", anonymous=True, disable_signals=True)
    try:
        if args.command == "text":
            rospy.wait_for_service("/vla/write_text", timeout=15.0)
            result = rospy.ServiceProxy("/vla/write_text", WriteText)(" ".join(args.value))
        else:
            path = os.path.abspath(os.path.expanduser(args.path))
            rospy.wait_for_service("/vla/draw_image", timeout=15.0)
            result = rospy.ServiceProxy("/vla/draw_image", DrawImage)(path)
    except Exception as exc:
        print("request failed: {}".format(exc), file=sys.stderr)
        return 1
    print("{}: {}".format("OK" if result.success else "FAILED", result.message))
    return 0 if result.success else 1


if __name__ == "__main__":
    sys.exit(main())
