#include "slam/msg_conversion.h"

Rigid3d FromRos(const geometry_msgs::PoseWithCovariance &pose_msg) {
  Rigid3d pose;
  pose.translation().x() = pose_msg.pose.position.x;
  pose.translation().y() = pose_msg.pose.position.y;
  pose.translation().z() = pose_msg.pose.position.z;
  pose.rotation().x()    = pose_msg.pose.orientation.x;
  pose.rotation().y()    = pose_msg.pose.orientation.y;
  pose.rotation().z()    = pose_msg.pose.orientation.z;
  pose.rotation().w()    = pose_msg.pose.orientation.w;
  return pose;
}

geometry_msgs::PoseWithCovariance ToRos(const Rigid3d &pose) {
  geometry_msgs::PoseWithCovariance pose_msg;
  pose_msg.pose.position.x    = pose.translation().x();
  pose_msg.pose.position.y    = pose.translation().y();
  pose_msg.pose.position.z    = pose.translation().z();
  pose_msg.pose.orientation.x = pose.rotation().x();
  pose_msg.pose.orientation.y = pose.rotation().y();
  pose_msg.pose.orientation.z = pose.rotation().z();
  pose_msg.pose.orientation.w = pose.rotation().w();
  return pose_msg;
}

Time FromRos(const ros::Time &time) {
  return FromUniversal(time.sec * UniversalTimeScaleClock::f1 +
                       time.nsec / UniversalTimeScaleClock::f2);
}

ros::Time ToRos(const Time &time) {
  int64_t t = ToUniversal(time);
  return {
      uint32_t(t / UniversalTimeScaleClock::f1),
      uint32_t(t % UniversalTimeScaleClock::f1 * UniversalTimeScaleClock::f2)};
}

proto::Vector3d ToProto(const Vector3d &v) {
  proto::Vector3d pv;
  pv.set_x(v.x());
  pv.set_y(v.y());
  pv.set_z(v.z());
  return pv;
}

proto::Quaterniond ToProto(const Quaterniond &v) {
  proto::Quaterniond pq;
  pq.set_x(v.x());
  pq.set_y(v.y());
  pq.set_z(v.z());
  pq.set_w(v.w());
  return pq;
}

Vector3d FromProto(const proto::Vector3d &pv) {
  return {pv.x(), pv.y(), pv.z()};
}

Quaterniond FromProto(const proto::Quaterniond &pv) {
  return {pv.w(), pv.x(), pv.y(), pv.z()};
}
