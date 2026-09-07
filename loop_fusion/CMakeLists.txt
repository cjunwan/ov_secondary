cmake_minimum_required(VERSION 2.8.3)
project(loop_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    camera_models
    cv_bridge
    roslib
    nlink_parser_ros2_interfaces
    message_generation
    )


set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
find_package(OpenCV 4.2 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

add_message_files(
    FILES
    RobotPairDistances.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES loop_fusion_lib
    CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs camera_models cv_bridge nlink_parser_ros2_interfaces message_runtime
)

catkin_install_python(PROGRAMS
    scripts/nlink_to_pair_distances.py
    scripts/initial_pose_to_world.py
    scripts/inject_vio_failure.py
    scripts/multi_robot_visualizer.py
    scripts/static_world_pose.py
    scripts/summarize_recovery_run.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(loop_fusion_lib
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
)
target_link_libraries(loop_fusion_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(loop_fusion_node src/pose_graph_node.cpp)
# loop_fusion_lib intentionally refers to a few globals owned by the node.
# Ubuntu's default --as-needed can otherwise discard the DSO before resolving
# the node's PoseGraph symbols in isolated/overlay builds.
target_link_libraries(loop_fusion_node -Wl,--no-as-needed loop_fusion_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(range_frame_aligner_node src/range_frame_aligner_node.cpp)
target_link_libraries(range_frame_aligner_node ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(local_frame_reset_node src/local_frame_reset_node.cpp)
target_link_libraries(local_frame_reset_node ${catkin_LIBRARIES})

add_dependencies(loop_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(loop_fusion_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(range_frame_aligner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(local_frame_reset_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_recovery_factors test/test_recovery_factors.cpp)
    if(TARGET test_recovery_factors)
        target_include_directories(test_recovery_factors PRIVATE src)
        target_link_libraries(test_recovery_factors ${catkin_LIBRARIES} ${CERES_LIBRARIES})
        add_dependencies(test_recovery_factors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif()
endif()
