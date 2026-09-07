#!/usr/bin/env python3
"""Publish a timestamped static robot pose for range-factor synchronization."""

import math

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("static_world_pose")
    topic = rospy.get_param("~topic", "/anchor/omo1")
    frame = rospy.get_param("~world_frame", "global")
    position = [float(value) for value in rospy.get_param("~position", [0.0, 0.0, 0.0])]
    yaw = math.radians(float(rospy.get_param("~yaw_deg", 0.0)))
    rate_hz = float(rospy.get_param("~rate", 30.0))
    if len(position) != 3 or not all(math.isfinite(value) for value in position):
        raise ValueError("~position must contain three finite values")
    if rate_hz <= 0.0:
        raise ValueError("~rate must be positive")

    publisher = rospy.Publisher(topic, PoseStamped, queue_size=10, latch=True)
    rate = rospy.Rate(rate_hz)
    message = PoseStamped()
    message.header.frame_id = frame
    message.pose.position.x, message.pose.position.y, message.pose.position.z = position
    message.pose.orientation.z = math.sin(0.5 * yaw)
    message.pose.orientation.w = math.cos(0.5 * yaw)
    rospy.loginfo("Static range anchor: %s p=%s yaw=%.3f deg", topic, position,
                  math.degrees(yaw))
    try:
        while not rospy.is_shutdown():
            message.header.stamp = rospy.Time.now()
            publisher.publish(message)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
