#!/usr/bin/env python3
"""Convert the merged ROS 2 sensor bags into a compact ROS 1 experiment bag.

Only the stereo/IMU streams required by OpenVINS are copied.  The NLink UWB
frames are converted to loop_fusion/RobotPairDistances, so the resulting bag
can be played directly into merged_recovery.launch.
"""

import argparse
import math
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Writer
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


NODE_DEFINITION = """\
uint8 role
uint8 id
float32 dis
float32 fp_rssi
float32 rx_rssi
"""

FRAME_DEFINITION = """\
uint8 role
uint8 id
builtin_interfaces/Time stamp
uint32 local_time
uint32 system_time
float32 voltage
float32[3] pos_3d
float32[3] eop_3d
float32[3] vel_3d
float32[3] angle_3d
float32[4] quaternion
float32[3] imu_gyro_3d
float32[3] imu_acc_3d
nlink_parser_ros2_interfaces/LinktrackNode2[] nodes
"""

DISTANCE_DEFINITION = """\
std_msgs/Header header
uint8 group_id
string leader_name
string[] robot_a_names
string[] robot_b_names
float64[] distances
"""

NODE_TYPE = "nlink_parser_ros2_interfaces/msg/LinktrackNode2"
FRAME_TYPE = "nlink_parser_ros2_interfaces/msg/LinktrackNodeframe2"
DISTANCE_TYPE = "loop_fusion/msg/RobotPairDistances"


def sensor_topics(robots):
    suffixes = (
        "camera/infra1/image_rect_raw",
        "camera/infra2/image_rect_raw",
        "camera/imu",
    )
    return {f"/{robot}/{suffix}" for robot in robots for suffix in suffixes}


def make_source_stores():
    source = get_typestore(Stores.ROS2_HUMBLE)
    source.register(get_types_from_msg(NODE_DEFINITION, NODE_TYPE))
    source.register(get_types_from_msg(FRAME_DEFINITION, FRAME_TYPE))

    # cdr_to_ros1 needs ROS 2 field layouts but a ROS 1 Header (with seq).
    ros1_wire = get_typestore(Stores.ROS2_HUMBLE)
    ros1_wire.register(get_types_from_msg(NODE_DEFINITION, NODE_TYPE))
    ros1_wire.register(get_types_from_msg(FRAME_DEFINITION, FRAME_TYPE))
    noetic = get_typestore(Stores.ROS1_NOETIC)
    ros1_wire.FIELDDEFS.pop("std_msgs/msg/Header")
    ros1_wire.register(
        {"std_msgs/msg/Header": noetic.FIELDDEFS["std_msgs/msg/Header"]}
    )
    return source, ros1_wire


def make_distance_store():
    store = get_typestore(Stores.ROS1_NOETIC)
    store.register(get_types_from_msg(DISTANCE_DEFINITION, DISTANCE_TYPE))
    return store


def frame_stamp_ns(frame, record_time_ns):
    stamp_ns = int(frame.stamp.sec) * 1_000_000_000 + int(frame.stamp.nanosec)
    return stamp_ns if stamp_ns > 0 else record_time_ns


def make_distance_message(store, frame, record_time_ns, id_to_name, sequence):
    self_name = id_to_name.get(int(frame.id))
    if self_name is None:
        return None

    a_names = []
    b_names = []
    distances = []
    for node in frame.nodes:
        other_name = id_to_name.get(int(node.id))
        distance = float(node.dis)
        if (
            other_name is None
            or other_name == self_name
            or not math.isfinite(distance)
            or distance <= 0.0
        ):
            continue
        a_names.append(self_name)
        b_names.append(other_name)
        distances.append(distance)

    if not distances:
        return None

    stamp_ns = frame_stamp_ns(frame, record_time_ns)
    Time = store.types["builtin_interfaces/msg/Time"]
    Header = store.types["std_msgs/msg/Header"]
    RobotPairDistances = store.types[DISTANCE_TYPE]
    header = Header(
        seq=sequence,
        stamp=Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000),
        frame_id="uwb",
    )
    return RobotPairDistances(
        header=header,
        group_id=1,
        leader_name=id_to_name.get(1, ""),
        robot_a_names=a_names,
        robot_b_names=b_names,
        distances=np.asarray(distances, dtype=np.float64),
    )


def convert(input_path, output_path, robots, id_to_name, max_messages):
    source_store, ros1_wire_store = make_source_stores()
    distance_store = make_distance_store()
    selected = sensor_topics(robots)
    uwb_topics = {f"/{name}/nlink_linktrack_nodeframe2" for name in id_to_name.values()}

    copied = 0
    distance_count = 0
    rejected_uwb = 0

    with Reader(input_path) as reader, Writer(output_path) as writer:
        source_connections = {
            connection.id: connection
            for connection in reader.connections
            if connection.topic in selected
        }
        missing = sorted(selected - {connection.topic for connection in source_connections.values()})
        if missing:
            raise RuntimeError("required topics are missing: " + ", ".join(missing))

        output_connections = {
            connection.id: writer.add_connection(
                connection.topic,
                connection.msgtype,
                typestore=ros1_wire_store,
                latching=0,
            )
            for connection in source_connections.values()
        }
        distance_connection = writer.add_connection(
            "/group1/structure_distances",
            DISTANCE_TYPE,
            typestore=distance_store,
            latching=0,
        )

        relevant = [
            connection
            for connection in reader.connections
            if connection.id in source_connections or connection.topic in uwb_topics
        ]
        for connection, record_time_ns, rawdata in reader.messages(connections=relevant):
            if connection.id in source_connections:
                data = source_store.cdr_to_ros1(rawdata, connection.msgtype)
                writer.write(output_connections[connection.id], record_time_ns, data)
                copied += 1
            else:
                frame = source_store.deserialize_cdr(rawdata, FRAME_TYPE)
                message = make_distance_message(
                    distance_store, frame, record_time_ns, id_to_name, distance_count
                )
                if message is None:
                    rejected_uwb += 1
                    continue
                data = distance_store.serialize_ros1(message, DISTANCE_TYPE)
                writer.write(distance_connection, record_time_ns, data)
                distance_count += 1

            if max_messages and copied + distance_count >= max_messages:
                break

    print(
        f"Wrote {copied} stereo/IMU messages and {distance_count} distance messages "
        f"to {output_path} ({rejected_uwb} empty/invalid UWB frames skipped)."
    )


def parse_id_map(value):
    result = {}
    for item in value.split(","):
        numeric_id, name = item.split(":", 1)
        result[int(numeric_id)] = name
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_ros2_bag", type=Path)
    parser.add_argument("output_ros1_bag", type=Path)
    parser.add_argument(
        "--robots",
        default="omo2,omo3,omo4",
        help="comma-separated robots whose stereo/IMU streams are copied",
    )
    parser.add_argument(
        "--id-map",
        default="1:omo1,2:omo2,3:omo3,4:omo4",
        help="comma-separated NLink ID:name mapping",
    )
    parser.add_argument(
        "--max-messages",
        type=int,
        default=0,
        help="stop early for a quick conversion test (0 means the whole bag)",
    )
    arguments = parser.parse_args()
    if arguments.output_ros1_bag.exists():
        raise RuntimeError(f"output already exists: {arguments.output_ros1_bag}")
    convert(
        arguments.input_ros2_bag,
        arguments.output_ros1_bag,
        [item for item in arguments.robots.split(",") if item],
        parse_id_map(arguments.id_map),
        arguments.max_messages,
    )


if __name__ == "__main__":
    main()
