#ifndef MSF_LOAM_VELODYNE_MSG_CONVERSION_H
#define MSF_LOAM_VELODYNE_MSG_CONVERSION_H

#include <geometry_msgs/PoseWithCovariance.h>

#include "common/rigid_transform.h"
#include "common/time.h"
#include "proto/msg.pb.h"

Rigid3d FromRos(const geometry_msgs::PoseWithCovariance &pose_msg);

geometry_msgs::PoseWithCovariance ToRos(const Rigid3d &pose);

Time FromRos(const ros::Time &time);

ros::Time ToRos(const Time &time);

proto::Vector3d ToProto(const Vector3d &v);

proto::Quaterniond ToProto(const Quaterniond &v);

Vector3d FromProto(const proto::Vector3d &pv);

Quaterniond FromProto(const proto::Quaterniond &pv);

#endif  // MSF_LOAM_VELODYNE_MSG_CONVERSION_H
