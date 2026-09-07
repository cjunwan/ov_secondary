#!/usr/bin/env python3
"""Anchor a local VIO pose stream at a known initial world pose."""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def rotate_z(vector, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        c * vector[0] - s * vector[1],
        s * vector[0] + c * vector[1],
        vector[2],
    )


def multiply_quaternions(a, b):
    # ROS order is x, y, z, w.
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_yaw(q):
    """Return ROS quaternion yaw in radians."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class InitialPoseToWorld:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic")
        self.output_topic = rospy.get_param("~output_topic")
        self.input_type = rospy.get_param(
            "~input_type", "PoseWithCovarianceStamped"
        )
        self.world_frame = rospy.get_param("~world_frame", "global")
        self.initial_position = [float(x) for x in rospy.get_param("~initial_position")]
        if len(self.initial_position) != 3 or not all(
            math.isfinite(x) for x in self.initial_position
        ):
            raise ValueError("~initial_position must contain three finite values")
        self.yaw = math.radians(float(rospy.get_param("~initial_yaw_deg", 0.0)))
        # Some ROS1 VIO outputs expose the camera/IMU frame orientation while
        # their position increments are interpreted as robot-body motion.  A
        # fixed planar correction lets us align the trajectory with the known
        # robot forward direction without changing the displayed orientation
        # or feeding anything back into the estimator.
        self.trajectory_yaw_correction = math.radians(float(rospy.get_param(
            "~trajectory_yaw_correction_deg", 0.0
        )))
        self.input_already_world_aligned = bool(rospy.get_param(
            "~input_already_world_aligned", False
        ))
        self.orientation_yaw_correction = math.radians(float(rospy.get_param(
            "~orientation_yaw_correction_deg", 0.0
        )))
        if not math.isfinite(self.trajectory_yaw_correction):
            raise ValueError("~trajectory_yaw_correction_deg must be finite")
        if not math.isfinite(self.orientation_yaw_correction):
            raise ValueError("~orientation_yaw_correction_deg must be finite")
        self.reference_position = None
        self.reference_yaw = None
        self.lock = threading.Lock()
        self.publisher = rospy.Publisher(self.output_topic, PoseStamped, queue_size=200)
        message_types = {
            "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
            "Odometry": Odometry,
        }
        if self.input_type not in message_types:
            raise ValueError(
                "~input_type must be PoseWithCovarianceStamped or Odometry"
            )
        self.subscriber = rospy.Subscriber(
            self.input_topic, message_types[self.input_type], self.callback,
            queue_size=500
        )
        rospy.loginfo(
            "Initial world pose: %s (%s) -> %s, p0=%s, yaw=%.3f deg, "
            "trajectory yaw correction=%.3f deg, orientation correction=%.3f deg, "
            "already world aligned=%s",
            self.input_topic,
            self.input_type,
            self.output_topic,
            self.initial_position,
            math.degrees(self.yaw),
            math.degrees(self.trajectory_yaw_correction),
            math.degrees(self.orientation_yaw_correction),
            self.input_already_world_aligned,
        )

    def callback(self, source):
        position = source.pose.pose.position
        local = (float(position.x), float(position.y), float(position.z))
        raw_q = source.pose.pose.orientation
        quaternion = (raw_q.x, raw_q.y, raw_q.z, raw_q.w)
        if not all(math.isfinite(x) for x in local + quaternion):
            return

        # Pose-graph output is already expressed in the configured world
        # frame. Do not latch and re-estimate another yaw transform: its
        # initial alignment can be refined after the first keyframe, which
        # would make a previously latched transform stale. For visualization,
        # rotate only the world position about the known initial pivot and
        # preserve the pose-graph orientation exactly.
        if self.input_already_world_aligned:
            relative = tuple(
                local[i] - self.initial_position[i] for i in range(3)
            )
            rotated = rotate_z(relative, self.trajectory_yaw_correction)
            body_q = (
                0.0,
                0.0,
                math.sin(self.orientation_yaw_correction / 2.0),
                math.cos(self.orientation_yaw_correction / 2.0),
            )
            # The pose graph's initial alignment was changed by a world-frame
            # yaw offset. Undo that offset for the RViz body arrow by composing
            # on the left. Right-composition would rotate about the tilted
            # camera/IMU local axis and gives a wrong heading when roll/pitch
            # are non-zero.
            q = multiply_quaternions(body_q, quaternion)
            self.publish(source, rotated, q)
            return

        local_yaw = quaternion_yaw(quaternion)
        with self.lock:
            if self.reference_position is None:
                self.reference_position = local
                self.reference_yaw = local_yaw
                rospy.loginfo(
                    "Latched first local VIO pose on %s: p=%s, yaw=%.3f deg, "
                    "world yaw offset=%.3f deg",
                    self.input_topic,
                    self.reference_position,
                    math.degrees(self.reference_yaw),
                    math.degrees(normalize_angle(self.yaw - self.reference_yaw)),
                )
            relative = tuple(
                local[i] - self.reference_position[i] for i in range(3)
            )
            yaw_offset = normalize_angle(self.yaw - self.reference_yaw)
        # Keep the pose arrow aligned by yaw_offset, but allow the trajectory
        # axes to account for a fixed camera/IMU-to-robot planar offset.
        rotated = rotate_z(
            relative,
            normalize_angle(yaw_offset + self.trajectory_yaw_correction),
        )

        yaw_q = (
            0.0, 0.0, math.sin(yaw_offset / 2.0), math.cos(yaw_offset / 2.0)
        )
        q = multiply_quaternions(yaw_q, quaternion)

        self.publish(source, rotated, q)

    def publish(self, source, rotated, quaternion):
        target = PoseStamped()
        target.header = source.header
        target.header.frame_id = self.world_frame
        target.pose.position.x = self.initial_position[0] + rotated[0]
        target.pose.position.y = self.initial_position[1] + rotated[1]
        target.pose.position.z = self.initial_position[2] + rotated[2]
        target.pose.orientation.x = quaternion[0]
        target.pose.orientation.y = quaternion[1]
        target.pose.orientation.z = quaternion[2]
        target.pose.orientation.w = quaternion[3]
        self.publisher.publish(target)


def main():
    rospy.init_node("initial_pose_to_world")
    InitialPoseToWorld()
    rospy.spin()


if __name__ == "__main__":
    main()
