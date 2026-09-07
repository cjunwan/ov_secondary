# `vio_results.bag` recovery experiment

## What is in this bag

`/home/junwan/1.datasets/merged/vio_results.bag` contains:

- raw stereo and IMU for `omo2`, `omo3`, and `omo4`;
- native NLink node frames for `omo1` through `omo4`;
- previously computed `/ov_msckf_omo{2,3,4}/odomimu`.

It does **not** contain `loop_pose`, `loop_feats`, intrinsics/extrinsics, or
`RobotPairDistances`, so the launch reruns VIO from the raw streams and
converts NLink frames online. The recorded odometry is not used as a global
neighbor anchor: every robot starts near `(0,0,0)`, and the trajectories are
independently initialized local VIO frames despite the `global` frame label.

## Build

```bash
cd /home/junwan/microswarm_ws
catkin build nlink_parser_ros2_interfaces camera_models ov_core ov_init ov_msckf \
  loop_fusion failure_recovery
source devel/setup.bash
```

## Range-assisted run

```bash
roslaunch loop_fusion merged_recovery.launch \
  play_bag:=true \
  bag:=/home/junwan/1.datasets/merged/vio_results.bag \
  bag_rate:=0.5 \
  range_assisted:=true \
  inject_failure:=true \
  failure_delay:=30 \
  omo1_initial_x:=0.0 omo1_initial_y:=0.0 omo1_initial_yaw_deg:=0.0 \
  omo2_initial_x:=4.27 omo2_initial_y:=0.0 omo2_initial_yaw_deg:=90.0 \
  omo3_initial_x:=4.27 omo3_initial_y:=5.5 omo3_initial_yaw_deg:=180.0 \
  omo4_initial_x:=0.0 omo4_initial_y:=5.5 omo4_initial_yaw_deg:=-90.0 \
  rviz:=true
```

The injector starts before playback and uses the first non-zero `/clock`, so
`failure_delay` is a reproducible bag-time offset. The poses shown above use
the measured 4.27 m wide by 5.5 m tall rectangular layout, with each robot facing
counter-clockwise toward the next corner. For a manual reset, launch with
`inject_failure:=false` and run:

```bash
source /home/junwan/microswarm_ws/devel/setup.bash
rosrun loop_fusion inject_vio_failure.py --delay 0 --topic /omo2/restart
```

## Visual-only baseline

Restart all nodes, then run the same bag and reset delay with the range adapter
and range recovery disabled:

```bash
roslaunch loop_fusion merged_recovery.launch \
  play_bag:=true \
  bag:=/home/junwan/1.datasets/merged/vio_results.bag \
  bag_rate:=0.5 \
  range_assisted:=false \
  inject_failure:=true \
  failure_delay:=30 \
  omo2_initial_x:=4.27 omo2_initial_y:=0.0 omo2_initial_z:=0.0 \
  omo2_initial_yaw_deg:=90.0 \
  config_path:=$(rospack find loop_fusion)/../config/merged_omo2_visual_only.yaml
```

## Dataset limitation

This bag has no initial-offset-applied world pose topic. The launch therefore
anchors each newly computed local VIO stream at the initial world positions
provided on the command line. It publishes the neighbor inputs as
`/initial_world_pose/omo3` and `/initial_world_pose/omo4`; these are the exact
topics consumed by range recovery. In deployment, an upstream component may
publish equivalent world-frame poses directly on those topics.

The recorded VIO results also grow to physically impossible kilometre-scale
positions near the end of the bag. The launch deliberately reruns VIO from raw
stereo/IMU rather than using those recorded outputs. If a newly computed
neighbor VIO also diverges, range recovery must reject its inconsistent range
residual rather than force a trajectory stitch.
