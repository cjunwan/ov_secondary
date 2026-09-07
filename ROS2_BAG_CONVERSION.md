# Merged ROS 2 bag to ROS 1

The generic `rosbags-convert` command cannot read these bags because it does
not know the custom NLink type. The recorded schema also contains a `stamp`
field that is absent from the older interface source under
`microswarm_ros2`, so registering that old file still produces an incorrect
layout.

Use the project converter, which registers the exact recorded schema and
preserves every topic and bag-record timestamp:

```bash
cd /home/junwan/1.datasets/merged

python3 /home/junwan/microswarm_ws/src/ov_secondary/loop_fusion/scripts/convert_ros2_to_ros1.py \
  trial1 trial1_ros1.bag
python3 /home/junwan/microswarm_ws/src/ov_secondary/loop_fusion/scripts/convert_ros2_to_ros1.py \
  trial2 trial2_ros1.bag
python3 /home/junwan/microswarm_ws/src/ov_secondary/loop_fusion/scripts/convert_ros2_to_ros1.py \
  trial3 trial3_ros1.bag
```

LZ4 compression is enabled by default. Use `--compression none` only when an
uncompressed bag is specifically required. The converter refuses to overwrite
an existing destination.

For a short conversion check:

```bash
python3 /home/junwan/microswarm_ws/src/ov_secondary/loop_fusion/scripts/convert_ros2_to_ros1.py \
  trial1 /tmp/trial1_smoke.bag --max-messages 5000
rosbag info /tmp/trial1_smoke.bag
```

## Build the ROS 1 NLink messages

The bag embeds its message definition, so `rosbag info` works immediately.
ROS 1 nodes that subscribe to the NLink topics also need the corresponding
generated message class. This repository now provides it:

```bash
cd /home/junwan/microswarm_ws
catkin build nlink_parser_ros2_interfaces
source devel/setup.bash

rosmsg show nlink_parser_ros2_interfaces/LinktrackNodeframe2
rosbag play --clock /home/junwan/1.datasets/merged/trial1_ros1.bag
```

If only VIO input is needed and disk space is tight, color and point-cloud
topics can be omitted with repeated `--exclude TOPIC` options. Infrared stereo,
IMU, and all four NLink topics should remain for the planned recovery test.
