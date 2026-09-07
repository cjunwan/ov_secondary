uint8 role
uint8 id

# The recorded ROS 2 type contains this stamp even though an older source copy
# of the interface in microswarm_ros2 does not. ROS 1 `time` has the same
# seconds/nanoseconds wire representation for these recordings.
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
