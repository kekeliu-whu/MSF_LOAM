cmake_minimum_required(VERSION 2.8.3)
project(aloam_velodyne)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)


add_executable(msf_loam_node
  src/slam/scan_registration.cc
  src/slam/laser_mapping.cc
  src/slam/laser_odometry.cc
  src/slam/scan_matching/odometry_scan_matcher.cc
  src/slam/scan_matching/mapping_scan_matcher.cc
  src/slam/hybrid_grid.cc)
target_link_libraries(msf_loam_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES})

add_executable(kittiHelper src/slam/kitti_helper.cc)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
