//
// Created by whu on 12/16/19.
//

#ifndef ALOAM_VELODYNE_TYPE_CONVERSION_H
#define ALOAM_VELODYNE_TYPE_CONVERSION_H

#include <geometry_msgs/PoseWithCovariance.h>

#include "common/rigid_transform.h"

inline Rigid3d FromRos(const geometry_msgs::PoseWithCovariance &pose_msg) {
  Rigid3d pose;
  pose.translation().x() = pose_msg.pose.position.x;
  pose.translation().y() = pose_msg.pose.position.y;
  pose.translation().z() = pose_msg.pose.position.z;
  pose.rotation().x() = pose_msg.pose.orientation.x;
  pose.rotation().y() = pose_msg.pose.orientation.y;
  pose.rotation().z() = pose_msg.pose.orientation.z;
  pose.rotation().w() = pose_msg.pose.orientation.w;
  return pose;
}

inline geometry_msgs::PoseWithCovariance ToRos(const Rigid3d &pose) {
  geometry_msgs::PoseWithCovariance pose_msg;
  pose_msg.pose.position.x = pose.translation().x();
  pose_msg.pose.position.y = pose.translation().y();
  pose_msg.pose.position.z = pose.translation().z();
  pose_msg.pose.orientation.x = pose.rotation().x();
  pose_msg.pose.orientation.y = pose.rotation().y();
  pose_msg.pose.orientation.z = pose.rotation().z();
  pose_msg.pose.orientation.w = pose.rotation().w();
  return pose_msg;
}

#endif  // ALOAM_VELODYNE_TYPE_CONVERSION_H
