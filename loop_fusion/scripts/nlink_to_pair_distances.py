#!/usr/bin/env python3
"""Convert recorded NLink node frames to RobotPairDistances in real time."""

import math

import rospy
from loop_fusion.msg import RobotPairDistances
from nlink_parser_ros2_interfaces.msg import LinktrackNodeframe2


class NLinkDistanceAdapter:
    def __init__(self):
        self.output_topic = rospy.get_param(
            "~output_topic", "/group1/structure_distances"
        )
        self.input_topics = rospy.get_param(
            "~input_topics",
            [
                "/omo1/nlink_linktrack_nodeframe2",
                "/omo2/nlink_linktrack_nodeframe2",
                "/omo3/nlink_linktrack_nodeframe2",
                "/omo4/nlink_linktrack_nodeframe2",
            ],
        )
        raw_id_map = rospy.get_param(
            "~id_to_name", {"1": "omo1", "2": "omo2", "3": "omo3", "4": "omo4"}
        )
        self.id_to_name = {int(key): str(value) for key, value in raw_id_map.items()}
        self.group_id = int(rospy.get_param("~group_id", 1))
        self.leader_name = str(rospy.get_param("~leader_name", "omo1"))
        self.max_distance = float(rospy.get_param("~max_distance", 50.0))

        self.publisher = rospy.Publisher(
            self.output_topic, RobotPairDistances, queue_size=500
        )
        self.subscribers = [
            rospy.Subscriber(topic, LinktrackNodeframe2, self.callback, queue_size=500)
            for topic in self.input_topics
        ]
        self.rejected = 0
        rospy.loginfo(
            "NLink adapter: %s -> %s", ", ".join(self.input_topics), self.output_topic
        )

    def callback(self, source):
        self_name = self.id_to_name.get(int(source.id))
        if self_name is None:
            rospy.logwarn_throttle(2.0, "Unknown NLink sender ID: %d", source.id)
            return

        target = RobotPairDistances()
        target.header.stamp = source.stamp if not source.stamp.is_zero() else rospy.Time.now()
        target.header.frame_id = "uwb"
        target.group_id = self.group_id
        target.leader_name = self.leader_name

        for node in source.nodes:
            other_name = self.id_to_name.get(int(node.id))
            distance = float(node.dis)
            if (
                other_name is None
                or other_name == self_name
                or not math.isfinite(distance)
                or distance <= 0.0
                or distance > self.max_distance
            ):
                self.rejected += 1
                continue
            target.robot_a_names.append(self_name)
            target.robot_b_names.append(other_name)
            target.distances.append(distance)

        if target.distances:
            self.publisher.publish(target)


def main():
    rospy.init_node("nlink_to_pair_distances")
    NLinkDistanceAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
