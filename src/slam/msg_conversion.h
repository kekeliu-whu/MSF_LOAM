#ifndef MSF_LOAM_VELODYNE_MSG_CONVERSION_H
#define MSF_LOAM_VELODYNE_MSG_CONVERSION_H

#include <geometry_msgs/PoseWithCovariance.h>

#include "common/rigid_transform.h"
#include "common/time_def.h"

Rigid3d FromRos(const geometry_msgs::PoseWithCovariance &pose_msg);

geometry_msgs::PoseWithCovariance ToRos(const Rigid3d &pose);

Time FromRos(const ros::Time &time);

ros::Time ToRos(const Time &time);

#endif  // MSF_LOAM_VELODYNE_MSG_CONVERSION_H
