#!/usr/bin/env python3
"""Publish distinct RViz bodies, labels and trails for the four robots."""

import copy
import math
import threading

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


COLORS = {
    "omo1": (0.95, 0.15, 0.15),  # red
    "omo2": (0.10, 0.90, 0.20),  # green
    "omo3": (0.10, 0.40, 1.00),  # blue
    "omo4": (0.95, 0.15, 0.90),  # magenta
}


def initial_pose(position, yaw_degrees=0.0):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [float(v) for v in position]
    yaw = math.radians(float(yaw_degrees))
    pose.orientation.z = math.sin(0.5 * yaw)
    pose.orientation.w = math.cos(0.5 * yaw)
    return pose


class MultiRobotVisualizer:
    def __init__(self):
        self.frame = rospy.get_param("~world_frame", "global")
        self.max_trail_points = int(rospy.get_param("~max_trail_points", 10000))
        self.min_trail_step = float(rospy.get_param("~min_trail_step", 0.02))
        self.lock = threading.Lock()
        self.poses = {
            name: initial_pose(
                rospy.get_param("~{}_initial_position".format(name)),
                rospy.get_param("~{}_initial_yaw_deg".format(name), 0.0),
            )
            for name in COLORS
        }
        self.trails = {name: [] for name in COLORS}
        for name, pose in self.poses.items():
            self._append_trail(name, pose.position)

        self.publisher = rospy.Publisher(
            "/recovery/robot_markers", MarkerArray, queue_size=10, latch=True
        )
        # Per-robot comparison: retain the last unstitched combined path when
        # recovery completes, while pose graph continues publishing the
        # corrected combined path on a separate display topic.
        self.recovery_robots = ("omo2", "omo3", "omo4")
        self.pre_merge_paths = {name: Path() for name in self.recovery_robots}
        self.recovery_active = {name: False for name in self.recovery_robots}
        self.publish_post_merge = {name: True for name in self.recovery_robots}
        self.pre_merge_publishers = {
            name: rospy.Publisher(
                "/recovery_comparison/{}/pre_merge_path".format(name),
                Path,
                queue_size=2,
                latch=True,
            )
            for name in self.recovery_robots
        }
        self.post_merge_publishers = {
            name: rospy.Publisher(
                "/recovery_comparison/{}/post_merge_path".format(name),
                Path,
                queue_size=2,
                latch=True,
            )
            for name in self.recovery_robots
        }
        message_types = {
            "PoseStamped": PoseStamped,
            "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
            "Odometry": Odometry,
        }
        self.subscribers = []
        for name in ("omo2", "omo3", "omo4"):
            topic = rospy.get_param(
                "~{}_topic".format(name), "/ov_secondary_{}/poseimu".format(name)
            )
            type_name = rospy.get_param(
                "~{}_pose_type".format(name), "PoseWithCovarianceStamped"
            )
            if type_name not in message_types:
                raise ValueError("Unsupported pose type for {}: {}".format(name, type_name))
            self.subscribers.append(rospy.Subscriber(
                topic, message_types[type_name], self.pose_message_callback,
                callback_args=(name, type_name), queue_size=200,
            ))
        for name in self.recovery_robots:
            self.subscribers.extend([
                rospy.Subscriber(
                    "/{}/recovery_restart".format(name), Bool,
                    self.restart_callback, callback_args=name, queue_size=10,
                ),
                rospy.Subscriber(
                    "/loop_fusion_{}/combined_path".format(name), Path,
                    self.combined_path_callback, callback_args=name, queue_size=20,
                ),
                rospy.Subscriber(
                    "/{}/failure_recovery/trigger".format(name), Bool,
                    self.recovery_trigger_callback,
                    callback_args=name,
                    queue_size=10,
                ),
            ])
        rospy.Timer(rospy.Duration(0.2), self.timer_callback)
        rospy.loginfo("Four-robot RViz markers: omo1=red, omo2=green, omo3=blue, omo4=magenta")

    def _append_trail(self, name, position):
        point = Point(position.x, position.y, position.z)
        trail = self.trails[name]
        if trail:
            previous = trail[-1]
            distance = math.sqrt(
                (point.x - previous.x) ** 2
                + (point.y - previous.y) ** 2
                + (point.z - previous.z) ** 2
            )
            if distance < self.min_trail_step:
                return
        trail.append(point)
        if len(trail) > self.max_trail_points:
            del trail[:len(trail) - self.max_trail_points]

    def update(self, name, pose):
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return
        with self.lock:
            self.poses[name] = copy.deepcopy(pose)
            self._append_trail(name, pose.position)

    def pose_message_callback(self, message, callback_args):
        name, type_name = callback_args
        pose = message.pose if type_name == "PoseStamped" else message.pose.pose
        self.update(name, pose)

    def restart_callback(self, message, name):
        if not message.data:
            return
        empty = Path()
        empty.header.frame_id = self.frame
        empty.header.stamp = rospy.Time.now()
        with self.lock:
            self.recovery_active[name] = True
            self.publish_post_merge[name] = False
            self.pre_merge_paths[name] = empty
        self.pre_merge_publishers[name].publish(empty)
        self.post_merge_publishers[name].publish(empty)
        rospy.loginfo("%s pre-merge trajectory capture started", name)

    def combined_path_callback(self, message, name):
        with self.lock:
            if self.recovery_active[name]:
                self.pre_merge_paths[name] = copy.deepcopy(message)
                pre_merge = copy.deepcopy(self.pre_merge_paths[name])
                post_merge = None
            elif self.publish_post_merge[name]:
                pre_merge = None
                post_merge = copy.deepcopy(message)
            else:
                return
        if pre_merge is not None:
            self.pre_merge_publishers[name].publish(pre_merge)
        if post_merge is not None:
            self.post_merge_publishers[name].publish(post_merge)

    def freeze_pre_merge_path(self, name, reason):
        with self.lock:
            if not self.recovery_active[name]:
                return
            self.recovery_active[name] = False
            # The next combined_path update is produced after pose-graph
            # stitching and becomes the green post-merge result.
            self.publish_post_merge[name] = True
            snapshot = copy.deepcopy(self.pre_merge_paths[name])
        self.pre_merge_publishers[name].publish(snapshot)
        rospy.loginfo(
            "%s pre-merge trajectory frozen (%s, %d poses)",
            name,
            reason,
            len(snapshot.poses),
        )

    def recovery_trigger_callback(self, message, name):
        if not message.data:
            self.freeze_pre_merge_path(name, "recovery completed or cancelled")

    def marker(self, name, marker_id, marker_type, now):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = now
        marker.ns = "multi_robot_{}".format(name)
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b = COLORS[name]
        marker.color.a = 1.0
        return marker

    def timer_callback(self, _event):
        now = rospy.Time.now()
        output = MarkerArray()
        with self.lock:
            for index, name in enumerate(("omo1", "omo2", "omo3", "omo4")):
                pose = self.poses[name]
                base = index * 10

                body = self.marker(name, base, Marker.SPHERE, now)
                body.pose = copy.deepcopy(pose)
                body.scale.x = body.scale.y = 0.32
                body.scale.z = 0.20
                output.markers.append(body)

                heading = self.marker(name, base + 1, Marker.ARROW, now)
                heading.pose = copy.deepcopy(pose)
                heading.scale.x = 0.65
                heading.scale.y = 0.13
                heading.scale.z = 0.13
                output.markers.append(heading)

                label = self.marker(name, base + 2, Marker.TEXT_VIEW_FACING, now)
                label.pose.position.x = pose.position.x
                label.pose.position.y = pose.position.y
                label.pose.position.z = pose.position.z + 0.48
                label.scale.z = 0.30
                label.text = name
                output.markers.append(label)

                trail = self.marker(name, base + 3, Marker.LINE_STRIP, now)
                # Keep the live robot trail visually secondary to the actual
                # pose-graph paths configured in RViz.
                trail.scale.x = 0.025 if name == "omo1" else 0.015
                if name in self.recovery_robots:
                    trail.color.r = trail.color.g = trail.color.b = 0.65
                    trail.color.a = 0.55
                trail.points = copy.deepcopy(self.trails[name])
                output.markers.append(trail)
        self.publisher.publish(output)


def main():
    rospy.init_node("multi_robot_visualizer")
    MultiRobotVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
