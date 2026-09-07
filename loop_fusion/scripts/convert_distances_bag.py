#!/usr/bin/env python3
"""Convert a ROS 2 RobotPairDistances topic to a ROS 1 bag.

The custom ROS 2 type is registered from the message definition below, so this
script does not depend on a dataset-specific Python module.
"""

import argparse
import math

import rosbag
import rospy
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


MESSAGE_DEFINITION = """\
std_msgs/Header header
uint8 group_id
string leader_name
string[] robot_a_names
string[] robot_b_names
float64[] distances
"""


def message_time(msg, record_time_ns, use_record_time):
    if not use_record_time:
        stamp = msg.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns > 0:
            return rospy.Time(stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000)
    return rospy.Time(record_time_ns // 1_000_000_000, record_time_ns % 1_000_000_000)


def convert_distances_bag(input_path, output_path, topic, use_record_time):
    try:
        from loop_fusion.msg import RobotPairDistances
    except ImportError as error:
        raise RuntimeError("source the catkin workspace devel/setup.bash first") from error

    typestore = get_typestore(Stores.LATEST)
    count = 0
    rejected = 0
    registered_types = set()
    print(f"Converting {input_path} -> {output_path}, topic={topic}")

    with rosbag.Bag(output_path, "w") as output_bag, Reader(input_path) as reader:
        for connection, record_time_ns, rawdata in reader.messages():
            if connection.topic != topic:
                continue
            if connection.msgtype not in registered_types:
                typestore.register(get_types_from_msg(MESSAGE_DEFINITION, connection.msgtype))
                registered_types.add(connection.msgtype)
            source = typestore.deserialize_cdr(rawdata, connection.msgtype)

            a_names = list(source.robot_a_names)
            b_names = list(source.robot_b_names)
            distances = [float(value) for value in source.distances]
            if len(a_names) != len(b_names) or len(a_names) != len(distances):
                rejected += 1
                continue
            if any(not math.isfinite(value) or value <= 0.0 for value in distances):
                rejected += 1
                continue

            target = RobotPairDistances()
            target.header.stamp = message_time(source, record_time_ns, use_record_time)
            target.header.frame_id = source.header.frame_id
            target.group_id = int(source.group_id)
            target.leader_name = source.leader_name
            target.robot_a_names = a_names
            target.robot_b_names = b_names
            target.distances = distances
            output_bag.write(topic, target, target.header.stamp)
            count += 1

    print(f"Done: wrote {count} messages, rejected {rejected} malformed messages")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_ros2_bag", help="ROS 2 bag directory")
    parser.add_argument("output_ros1_bag", help="output .bag path")
    parser.add_argument("--topic", default="/group1/structure_distances")
    parser.add_argument(
        "--record-time",
        action="store_true",
        help="replace source header stamps with ROS 2 bag record times",
    )
    arguments = parser.parse_args()
    convert_distances_bag(
        arguments.input_ros2_bag,
        arguments.output_ros1_bag,
        arguments.topic,
        arguments.record_time,
    )


if __name__ == "__main__":
    main()
