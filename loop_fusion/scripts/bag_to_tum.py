#!/usr/bin/env python
"""
Extract trajectory from rosbag to TUM format for evo evaluation.

Supports:
  - nav_msgs/Path          (takes the LAST published message, which contains full history)
  - geometry_msgs/PoseStamped
  - nav_msgs/Odometry

Usage:
  python bag_to_tum.py <bag_file> <topic> <output_tum_file>

Example:
  python bag_to_tum.py test.bag /loop_fusion_node/pose_graph_path pg_path.tum
  python bag_to_tum.py test.bag /vrpn_mocap/driving/pose gt.tum
"""
import sys
import rosbag

def main():
    if len(sys.argv) != 4:
        print("Usage: {} <bag_file> <topic> <output.tum>".format(sys.argv[0]))
        sys.exit(1)

    bag_file = sys.argv[1]
    topic = sys.argv[2]
    output = sys.argv[3]

    bag = rosbag.Bag(bag_file, 'r')

    # Detect message type
    topic_info = bag.get_type_and_topic_info()
    if topic not in topic_info.topics:
        print("ERROR: topic '{}' not found in bag.".format(topic))
        print("Available topics:")
        for t in sorted(topic_info.topics.keys()):
            print("  {} [{}]".format(t, topic_info.topics[t].msg_type))
        sys.exit(1)

    msg_type = topic_info.topics[topic].msg_type
    print("Topic: {}  Type: {}".format(topic, msg_type))

    poses = []  # list of (timestamp_sec, x, y, z, qx, qy, qz, qw)

    if msg_type == "nav_msgs/Path":
        # Path: take the LAST message (it contains the complete trajectory)
        last_msg = None
        for _, msg, _ in bag.read_messages(topics=[topic]):
            last_msg = msg
        if last_msg is None:
            print("ERROR: no messages on topic '{}'".format(topic))
            sys.exit(1)
        for ps in last_msg.poses:
            t = ps.header.stamp.to_sec()
            p = ps.pose.position
            q = ps.pose.orientation
            poses.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        print("Extracted {} poses from Path (last message)".format(len(poses)))

    elif msg_type == "geometry_msgs/PoseStamped":
        for _, msg, _ in bag.read_messages(topics=[topic]):
            t = msg.header.stamp.to_sec()
            p = msg.pose.position
            q = msg.pose.orientation
            poses.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        print("Extracted {} poses from PoseStamped".format(len(poses)))

    elif msg_type == "nav_msgs/Odometry":
        for _, msg, _ in bag.read_messages(topics=[topic]):
            t = msg.header.stamp.to_sec()
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            poses.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        print("Extracted {} poses from Odometry".format(len(poses)))

    elif msg_type == "geometry_msgs/PoseWithCovarianceStamped":
        for _, msg, _ in bag.read_messages(topics=[topic]):
            t = msg.header.stamp.to_sec()
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            poses.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        print("Extracted {} poses from PoseWithCovarianceStamped".format(len(poses)))

    else:
        print("ERROR: unsupported message type: {}".format(msg_type))
        sys.exit(1)

    bag.close()

    # Write TUM format: timestamp x y z qx qy qz qw
    with open(output, 'w') as f:
        for row in poses:
            f.write("{:.9f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(*row))

    print("Saved to: {}".format(output))

if __name__ == '__main__':
    main()
