# VIO failure recovery experiment

## Preconditions

`RobotPairDistances` alone does not define an absolute pose. Every configured
neighbor pose must already be expressed in the same `global` frame as the
pre-failure trajectory. Raw VIO poses that independently start at the origin
are not valid neighbor anchors. At least one robot/trajectory must therefore
anchor the common frame.

The supplied `vio_results_1.bag` is an output/evaluation bag. It contains only
three `poseimu` topics and three `nlink_linktrack_nodeframe2` topics; it has no
images, IMU, loop features, camera calibration, or `RobotPairDistances` topic.
It cannot replay the complete recovery pipeline by itself.

## Isolated build

The current workspace `build/` and `devel/` directories may be owned by another
user. The following build avoids them:

```bash
mkdir -p /tmp/microswarm_recovery_ws
cd /tmp/microswarm_recovery_ws
catkin_make --source /home/junwan/microswarm_ws/src \
  -DCATKIN_WHITELIST_PACKAGES="camera_models;loop_fusion;failure_recovery" -j4
source /tmp/microswarm_recovery_ws/devel/setup.bash
```

Run the synthetic recovery tests:

```bash
cd /tmp/microswarm_recovery_ws
catkin_make run_tests_loop_fusion
catkin_test_results
```

## Input check

Before the range-assisted run, verify the live types, common frame, and rates:

```bash
rostopic type /group1/structure_distances
rostopic echo -n 1 /group1/structure_distances
rostopic type /UAV2/ov_secondary/poseimu
rostopic echo -n 1 /UAV2/ov_secondary/poseimu/header
rostopic hz /group1/structure_distances
rostopic hz /UAV2/ov_secondary/poseimu
```

Expected distance type is `loop_fusion/RobotPairDistances`. The three arrays
must have equal length. Neighbor headers must report `frame_id: global` (a
leading slash is accepted). For a bag whose sensor headers are inconsistent,
set `distance_use_header_timestamps: 0` so every input uses receipt time.

## A/B configuration

Copy `config/master_config.yaml` twice. Keep `robot_name: ""` in the baseline:

```bash
cp /home/junwan/microswarm_ws/src/ov_secondary/config/master_config.yaml /tmp/recovery_visual.yaml
cp /home/junwan/microswarm_ws/src/ov_secondary/config/master_config.yaml /tmp/recovery_visual_range.yaml
```

In `/tmp/recovery_visual_range.yaml`, set the exact robot identity and parallel
neighbor lists, for example:

```yaml
robot_name: "UAV1"
neighbor_names: ["UAV2", "UAV4"]
neighbor_pose_topics: ["/UAV2/ov_secondary/poseimu", "/UAV4/ov_secondary/poseimu"]
neighbor_pose_types: ["PoseWithCovarianceStamped", "PoseWithCovarianceStamped"]
```

Do not use `/ov_msckf_uav2/poseimu` or `/ov_msckf_uav4/poseimu` here unless a
separate fusion stage has demonstrably transformed them into the same global
frame and changed their `frame_id` accordingly.

## Run both trials

Start the complete VIO/sensor pipeline first. In another terminal, launch the
visual-only baseline:

```bash
source /tmp/microswarm_recovery_ws/devel/setup.bash
roslaunch loop_fusion posegraph.launch config_path:=/tmp/recovery_visual.yaml
```

Record the result and recovery events:

```bash
rosbag record -O /tmp/recovery_visual_result.bag \
  /loop_fusion_node/combined_path /ov_secondary/poseimu \
  /restart /ov_msckf/vio_reset /failure_recovery/recovered_pose
```

Inject the failure at a reproducible ROS-time offset (the VIO config must have
`failure_recovery_enabled: true`):

```bash
python3 /home/junwan/microswarm_ws/src/ov_secondary/loop_fusion/scripts/inject_vio_failure.py --delay 60
```

Repeat from a clean launch with `/tmp/recovery_visual_range.yaml`, recording to
`/tmp/recovery_visual_range_result.bag`. Use the same input bag, playback rate,
and failure delay. During a range run, an accepted fallback prints
`Range-only fallback accepted`; a visual match with usable ranges prints
`Combined recovery accepted`.

## Relative-recovery audit CSV

Each pose-graph process creates a timestamped audit file under
`/home/junwan/microswarm_ws/logs/relative_recovery/`. The important distinction
is:

- `RANGE_RECEIVED`: a valid robot-pair distance reached this robot.
- `RANGE_WAITING`: distances arrived but synchronized factors/geometry were not
  sufficient yet.
- `RANGE_OPTIMIZATION_ACCEPTED` or `RANGE_OPTIMIZATION_REJECTED`: range factors
  were actually passed to the recovery optimizer.
- `RECOVERY_COMPLETE` with `used_in_final_recovery=1`: range information
  affected the final stitched trajectory. This is the definitive success flag.

List the latest logs and check final range use:

```bash
ls -1t /home/junwan/microswarm_ws/logs/relative_recovery/*.csv | head
awk -F, 'NR == 1 || $13 == 1' \
  /home/junwan/microswarm_ws/logs/relative_recovery/*.csv
```

Inspect received measurements and rejected factors:

```bash
awk -F, 'NR == 1 || $4 == "RANGE_RECEIVED"' \
  /home/junwan/microswarm_ws/logs/relative_recovery/*.csv
awk -F, 'NR == 1 || $4 ~ /REJECTED/' \
  /home/junwan/microswarm_ws/logs/relative_recovery/*.csv
```

## Compare trajectories

Extract the final combined paths and evaluate both against the same ground
truth trajectory:

```bash
python3 /home/junwan/microswarm_ws/src/ov_secondary/loop_fusion/scripts/bag_to_tum.py \
  /tmp/recovery_visual_result.bag /loop_fusion_node/combined_path /tmp/visual.tum
python3 /home/junwan/microswarm_ws/src/ov_secondary/loop_fusion/scripts/bag_to_tum.py \
  /tmp/recovery_visual_range_result.bag /loop_fusion_node/combined_path /tmp/visual_range.tum
evo_ape tum /tmp/ground_truth.tum /tmp/visual.tum --align
evo_ape tum /tmp/ground_truth.tum /tmp/visual_range.tum --align
evo_rpe tum /tmp/ground_truth.tum /tmp/visual.tum --align
evo_rpe tum /tmp/ground_truth.tum /tmp/visual_range.tum --align
```

Compare recovery success rate, time from reset to accepted recovery, APE/RPE,
and the position/orientation discontinuity around the reset. Run several reset
times; a single trial is not enough to separate geometry from chance visual
matches.

## ROS 2 custom-distance bag conversion

The converter self-registers the custom type and preserves the source header
timestamp by default:

```bash
source /tmp/microswarm_recovery_ws/devel/setup.bash
python3 /home/junwan/microswarm_ws/src/ov_secondary/loop_fusion/scripts/convert_distances_bag.py \
  /path/to/ros2_bag /tmp/distances_ros1.bag
```

Use `--record-time` only if all streams will be synchronized by bag/receipt
time and set `distance_use_header_timestamps: 0` in that run.
