#!/usr/bin/env python3
"""Publish one deterministic recovery/restart event after a ROS-time delay."""

import argparse
import time

import rospy
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--delay", type=float, required=True, help="seconds after the first valid ROS time")
    parser.add_argument("--topic", default="/restart")
    args = parser.parse_args(rospy.myargv()[1:])
    if args.delay < 0.0:
        parser.error("--delay must be non-negative")

    rospy.init_node("inject_vio_failure", anonymous=True)
    publisher = rospy.Publisher(args.topic, Bool, queue_size=1)
    while not rospy.is_shutdown() and rospy.Time.now().is_zero():
        rospy.sleep(0.01)
    start = rospy.Time.now()
    target = start + rospy.Duration.from_sec(args.delay)
    rospy.loginfo("Recovery/restart event scheduled at ROS time %.6f", target.to_sec())
    while not rospy.is_shutdown() and rospy.Time.now() < target:
        rospy.sleep(0.01)
    if rospy.is_shutdown():
        return
    # rospy does not expose the roscpp WallTime/WallDuration classes.  Use a
    # monotonic wall clock here so this connection wait also works while bag
    # time is paused or being reset.
    timeout = time.monotonic() + 2.0
    while publisher.get_num_connections() == 0 and time.monotonic() < timeout:
        rospy.rostime.wallsleep(0.01)
    publisher.publish(Bool(data=True))
    rospy.logwarn("Published recovery/restart event on %s at ROS time %.6f", args.topic, rospy.Time.now().to_sec())
    rospy.rostime.wallsleep(0.2)


if __name__ == "__main__":
    main()
