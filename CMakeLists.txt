cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(msf_loam_velodyne VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 14)

add_definitions(-D_SIM_GPS)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        rosbag
        std_msgs
        tf)

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
        include
        src
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
        DEPENDS EIGEN3 PCL
        INCLUDE_DIRS include
)


add_executable(msf_loam_node
        src/common/time_def.cc
        src/slam/hybrid_grid.cc
        src/slam/imu_fusion/imu_tracker.cc
        src/slam/gps_fusion/gps_fusion.cc
        src/slam/msg_conversion.cc
        src/slam/scan_registration.cc
        src/slam/local/laser_mapping.cc
        src/slam/local/laser_odometry.cc
        src/slam/local/scan_matching/odometry_scan_matcher.cc
        src/slam/local/scan_matching/mapping_scan_matcher.cc
        src/slam/local/scan_matching/scan_matcher.cc
        src/slam/loop_closure/sparse_pose_graph.cc
        src/slam/local/scan_matching/lidar_factor.cc)
target_link_libraries(msf_loam_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(msf_loam_gps_fusion_test
#  src/common/time_def.cc
#  src/slam/gps_fusion/gps_fusion.cc
#  src/slam/gps_fusion/gps_fusion_test.cc
#  )
#target_link_libraries(msf_loam_gps_fusion_test ${CERES_LIBRARIES} gtest)

add_executable(kitti_helper src/slam/kitti_helper.cc)
target_link_libraries(kitti_helper ${catkin_LIBRARIES} ${PCL_LIBRARIES})
