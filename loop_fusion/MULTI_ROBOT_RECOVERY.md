# Three-robot failure recovery experiment

`multi_robot_recovery.launch` runs live VIO and failure recovery for `omo2`,
`omo3`, and `omo4`. `omo1` is published as a stationary world-frame anchor.
Each recovering robot therefore uses three range anchors:

- `omo2`: `omo1`, `omo3`, `omo4`
- `omo3`: `omo1`, `omo2`, `omo4`
- `omo4`: `omo1`, `omo2`, `omo3`

The assumed initial world poses are:

| Robot | x [m] | y [m] | yaw [deg] |
|---|---:|---:|---:|
| omo1 | 0.0 | 0.0 | 0 |
| omo2 | 4.27 | 0.0 | 90 |
| omo3 | 4.27 | 5.5 | 180 |
| omo4 | 0.0 | 5.5 | -90 |

The validated `micro_omo` VIO configuration is kept unchanged during the bag
experiment. A real mid-motion OpenVINS reinitialization is not used because its
static initializer can fail or diverge once the robot is moving. Instead,
`simulate_local_frame_reset:=true` leaves the healthy estimator running and
rebases all three frame-dependent outputs (`poseimu`, `loop_pose`, and
`loop_feats`) into a fresh local frame. The pose graph therefore receives the
same kind of disconnected trajectory segment that a successful VIO restart
would create, without mixing estimator lifecycle behavior into the recovery
comparison.

The IMU data in this bag is approximately 200 Hz. The `update_rate: 200` entry
in the Kalibr IMU YAML is descriptive metadata in this codebase; OpenVINS does
not read it to resample or change integration. Integration uses the timestamps
of the IMU messages actually received. Consequently, avoiding dropped/backlogged
IMU messages and using `rosbag play --clock` matter more than that YAML number.

## Build

Run as the normal `junwan` user. Workspace files must not be owned by root.

```bash
cd /home/junwan/microswarm_ws
source /opt/ros/noetic/setup.bash
catkin build nlink_parser_ros2_interfaces camera_models ov_core ov_init ov_msckf loop_fusion failure_recovery --force-cmake
source devel/setup.bash
```

## Run

Terminal 1, visual-only recovery with an actual new local trajectory segment:

```bash
cd /home/junwan/microswarm_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch loop_fusion multi_robot_recovery.launch \
  rviz:=true \
  simulate_local_frame_reset:=true \
  use_distance_recovery:=false \
  inject_failure:=true failure_robot:=omo2 failure_delay:=45.0 \
  experiment_tag:=trial1_visual_frame_reset_omo2
```

For visual + three-anchor range recovery, keep the same failure robot and delay
and change only the recovery mode and tag:

```bash
roslaunch loop_fusion multi_robot_recovery.launch \
  rviz:=true \
  simulate_local_frame_reset:=true \
  use_distance_recovery:=true \
  inject_failure:=true failure_robot:=omo2 failure_delay:=45.0 \
  experiment_tag:=trial1_visual_range_frame_reset_omo2
```

Use the same playback rate and failure timestamp in both trials.

`simulate_local_frame_reset:=false` retains the earlier sequence-only smoke
test. In that mode the pose graph starts a new sequence but raw VIO coordinates
do not restart, so it is not a valid quantitative failure-recovery test.

Terminal 2:

```bash
cd /home/junwan/microswarm_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosbag play --clock -d 3 -r 0.25 \
  /home/junwan/1.datasets/merged/trial1_ros1.bag \
  --topics \
  /omo2/camera/infra1/image_rect_raw /omo2/camera/infra2/image_rect_raw /omo2/camera/imu \
  /omo3/camera/infra1/image_rect_raw /omo3/camera/infra2/image_rect_raw /omo3/camera/imu \
  /omo4/camera/infra1/image_rect_raw /omo4/camera/infra2/image_rect_raw /omo4/camera/imu \
  /omo1/nlink_linktrack_nodeframe2 /omo2/nlink_linktrack_nodeframe2 \
  /omo3/nlink_linktrack_nodeframe2 /omo4/nlink_linktrack_nodeframe2
```

The explicit topic list is important: it prevents the recorded VIO outputs in
the bag from being mixed with the three live VIO instances. Start at `-r 0.25`;
increase toward `0.5` only if all three VIO processes keep up.

When `inject_failure:=true` is used, no third terminal is required. For manual
injection after VIO initialization and after a useful pre-failure trajectory
has accumulated, launch with `inject_failure:=false` and publish:

```bash
rostopic pub -1 /omo2/recovery_restart std_msgs/Bool "data: true"
```

Use `/omo3/recovery_restart` or `/omo4/recovery_restart` for the other robots.
Do not publish `/omoX/vio_restart` in this bag experiment; that topic invokes
the real moving-state initializer. For comparable measurements, use a fresh
launch and bag replay for each target robot and keep the failure timestamp the
same.

## Sanity checks

```bash
rostopic hz /ov_secondary_omo2/poseimu
rostopic hz /ov_secondary_omo3/poseimu
rostopic hz /ov_secondary_omo4/poseimu
rostopic hz /anchor/omo1
rostopic hz /group1/structure_distances
rostopic hz /rebased_vio_omo2/poseimu
```

After the injected event, `poseimu` on the rebased topic should start close to
zero while `/ov_msckf_omo2/poseimu` remains continuous and bounded. The reset
audit is written to
`logs/frame_reset/<experiment_tag>/omo2_frame_reset.csv`. Range factor decisions
are independently written below
`logs/relative_recovery/<experiment_tag>/`.

RViz shows each robot's pre-failure path, post-restart path, combined path,
pose graph, recovery stitch, and a distinct colored robot marker.

## Experiment constraints

- `omo1` must remain stationary for the entire recording. If it moves, replace
  the static anchor with its live world-frame pose.
- Trigger only one robot at a time. A recovering robot needs the other two live
  VIO poses plus `omo1`; simultaneous failures make those anchors invalid and
  can make the range alignment unobservable or circular.
- Use at most one forced failure per robot in a launch. The visual recovery
  database deliberately preserves that robot's pre-failure map.
- A range-only pose cannot determine yaw from one instant. The implementation
  estimates trajectory alignment from a window of multi-anchor range factors;
  visual loop closures are fused when available.
