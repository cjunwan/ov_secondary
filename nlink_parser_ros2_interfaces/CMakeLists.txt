cmake_minimum_required(VERSION 3.0.2)
project(nlink_parser_ros2_interfaces)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES
  LinktrackNode2.msg
  LinktrackNodeframe2.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
