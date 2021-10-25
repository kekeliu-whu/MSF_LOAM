cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0048 NEW)
project(msf_loam_velodyne VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 14)

#add_definitions(-D_SIM_GPS)

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
find_package(Protobuf REQUIRED)
find_package(fmt REQUIRED)

if (Protobuf_VERBOSE)
    message(STATUS "Using Protocol Buffers ${protobuf_VERSION}")
endif ()

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/msg.proto)

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(
    src
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
    DEPENDS EIGEN3 PCL
)

set(
    PROJECT_SRCS
    src/common/time.cc
    src/slam/hybrid_grid.cc
    src/slam/imu_fusion/initial_ex_rotation.cc
    src/slam/imu_fusion/pose_local_parameterization.cc
    src/slam/imu_fusion/scan_undistortion.cc
    src/slam/msg_conversion.cc
    src/slam/gps_fusion/gps_fusion.cc
    src/slam/local/laser_mapping.cc
    src/slam/local/laser_odometry.cc
    src/slam/local/scan_matching/odometry_scan_matcher.cc
    src/slam/local/scan_matching/mapping_scan_matcher.cc
    src/slam/local/scan_matching/scan_matcher.cc
    src/slam/loop_closure/sparse_pose_graph.cc
    src/slam/local/scan_matching/lidar_factor.cc
)

add_executable(msf_loam_node
    src/msf_loam_node.cc
    ${PROJECT_SRCS}
    ${PROTO_SRCS}
)
target_link_libraries(msf_loam_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${Protobuf_LIBRARIES})

add_executable(calib_lidar_imu
    src/calib_lidar_imu.cc
    ${PROJECT_SRCS}
    ${PROTO_SRCS}
)
target_link_libraries(calib_lidar_imu ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${Protobuf_LIBRARIES} fmt)

add_executable(kitti_helper src/slam/kitti_helper.cc)
target_link_libraries(kitti_helper ${catkin_LIBRARIES} ${PCL_LIBRARIES})

include(cmake/google-test.cmake)
add_test_library_srcs(${PROJECT_SRCS})
enable_automatic_test_and_benchmark()
