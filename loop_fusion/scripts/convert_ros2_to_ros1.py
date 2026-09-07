#!/usr/bin/env python3
"""Convert the merged ROS 2 bags to ROS 1, including their custom NLink type.

Unlike the generic rosbags-convert command, this utility registers the exact
LinktrackNodeframe2 schema stored in trial1--trial3. All topics are retained by
default and their bag record timestamps are preserved.
"""

import argparse
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Writer
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


NODE_TYPE = "nlink_parser_ros2_interfaces/msg/LinktrackNode2"
FRAME_TYPE = "nlink_parser_ros2_interfaces/msg/LinktrackNodeframe2"

NODE_DEFINITION = """\
uint8 role
uint8 id
float32 dis
float32 fp_rssi
float32 rx_rssi
"""

# This is the schema actually serialized in the new bags. In particular, it
# includes `stamp`; the older .msg file in microswarm_ros2 is stale.
ROS2_FRAME_DEFINITION = """\
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

ROS1_FRAME_DEFINITION = """\
uint8 role
uint8 id
time stamp
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


def make_stores():
    source = get_typestore(Stores.ROS2_HUMBLE)
    source.register(get_types_from_msg(NODE_DEFINITION, NODE_TYPE))
    source.register(get_types_from_msg(ROS2_FRAME_DEFINITION, FRAME_TYPE))

    # Standard ROS 2 messages can be converted at the byte-layout level. The
    # destination store must use ROS 1's Header, which adds the `seq` field.
    standard_ros1_wire = get_typestore(Stores.ROS2_HUMBLE)
    noetic = get_typestore(Stores.ROS1_NOETIC)
    standard_ros1_wire.FIELDDEFS.pop("std_msgs/msg/Header")
    standard_ros1_wire.register(
        {"std_msgs/msg/Header": noetic.FIELDDEFS["std_msgs/msg/Header"]}
    )

    custom_ros1 = get_typestore(Stores.ROS1_NOETIC)
    custom_ros1.register(get_types_from_msg(NODE_DEFINITION, NODE_TYPE))
    custom_ros1.register(get_types_from_msg(ROS1_FRAME_DEFINITION, FRAME_TYPE))
    return source, standard_ros1_wire, custom_ros1


def convert_nlink(source_message, custom_store):
    Time = custom_store.types["builtin_interfaces/msg/Time"]
    Node = custom_store.types[NODE_TYPE]
    Frame = custom_store.types[FRAME_TYPE]
    nodes = [
        Node(
            role=int(node.role),
            id=int(node.id),
            dis=float(node.dis),
            fp_rssi=float(node.fp_rssi),
            rx_rssi=float(node.rx_rssi),
        )
        for node in source_message.nodes
    ]
    return Frame(
        role=int(source_message.role),
        id=int(source_message.id),
        stamp=Time(
            sec=int(source_message.stamp.sec),
            nanosec=int(source_message.stamp.nanosec),
        ),
        local_time=int(source_message.local_time),
        system_time=int(source_message.system_time),
        voltage=float(source_message.voltage),
        pos_3d=np.asarray(source_message.pos_3d, dtype=np.float32),
        eop_3d=np.asarray(source_message.eop_3d, dtype=np.float32),
        vel_3d=np.asarray(source_message.vel_3d, dtype=np.float32),
        angle_3d=np.asarray(source_message.angle_3d, dtype=np.float32),
        quaternion=np.asarray(source_message.quaternion, dtype=np.float32),
        imu_gyro_3d=np.asarray(source_message.imu_gyro_3d, dtype=np.float32),
        imu_acc_3d=np.asarray(source_message.imu_acc_3d, dtype=np.float32),
        nodes=nodes,
    )


def convert(input_path, output_path, include_topics, exclude_topics, max_messages,
            compression):
    source_store, standard_store, custom_store = make_stores()
    count = 0
    per_topic = {}

    writer = Writer(output_path)
    if compression != "none":
        writer.set_compression(
            Writer.CompressionFormat.LZ4
            if compression == "lz4"
            else Writer.CompressionFormat.BZ2
        )
    with Reader(input_path) as reader, writer:
        selected = [
            connection
            for connection in reader.connections
            if (not include_topics or connection.topic in include_topics)
            and connection.topic not in exclude_topics
        ]
        if not selected:
            raise RuntimeError("no topics selected")

        output_connections = {}
        for connection in selected:
            is_nlink = connection.msgtype == FRAME_TYPE
            output_connections[connection.id] = writer.add_connection(
                connection.topic,
                connection.msgtype,
                typestore=custom_store if is_nlink else standard_store,
                latching=0,
            )

        for connection, timestamp, rawdata in reader.messages(connections=selected):
            if connection.msgtype == FRAME_TYPE:
                source_message = source_store.deserialize_cdr(rawdata, FRAME_TYPE)
                target_message = convert_nlink(source_message, custom_store)
                data = custom_store.serialize_ros1(target_message, FRAME_TYPE)
            else:
                data = source_store.cdr_to_ros1(rawdata, connection.msgtype)
            writer.write(output_connections[connection.id], timestamp, data)
            count += 1
            per_topic[connection.topic] = per_topic.get(connection.topic, 0) + 1
            if count % 10000 == 0:
                print(f"converted {count} messages", flush=True)
            if max_messages and count >= max_messages:
                break

    print(f"Done: {count} messages -> {output_path}")
    for topic, topic_count in sorted(per_topic.items()):
        print(f"  {topic}: {topic_count}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_ros2_bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument("output_ros1_bag", type=Path, help="new ROS 1 .bag file")
    parser.add_argument("--include", action="append", default=[], metavar="TOPIC")
    parser.add_argument("--exclude", action="append", default=[], metavar="TOPIC")
    parser.add_argument(
        "--max-messages", type=int, default=0,
        help="stop early for a smoke test; 0 converts the complete bag",
    )
    parser.add_argument(
        "--compression", choices=("lz4", "bz2", "none"), default="lz4",
        help="ROS 1 chunk compression (default: lz4)",
    )
    args = parser.parse_args()
    if not args.input_ros2_bag.is_dir():
        parser.error(f"ROS 2 bag directory does not exist: {args.input_ros2_bag}")
    if args.output_ros1_bag.exists():
        parser.error(f"output already exists: {args.output_ros1_bag}")
    convert(
        args.input_ros2_bag,
        args.output_ros1_bag,
        set(args.include),
        set(args.exclude),
        args.max_messages,
        args.compression,
    )


if __name__ == "__main__":
    main()
