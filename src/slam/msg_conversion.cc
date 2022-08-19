#include "slam/msg_conversion.h"

Vector3d FromROS(const geometry_msgs::Vector3_<std::allocator<void>> &p) {
  Vector3d v;
  v.x() = p.x;
  v.y() = p.y;
  v.z() = p.z;
  return v;
}

Vector3d FromROS(const geometry_msgs::Point_<std::allocator<void>> &p) {
  Vector3d v;
  v.x() = p.x;
  v.y() = p.y;
  v.z() = p.z;
  return v;
}

Quaterniond FromROS(const geometry_msgs::Quaternion_<std::allocator<void>> &o) {
  Quaterniond q;
  q.x() = o.x;
  q.y() = o.y;
  q.z() = o.z;
  q.w() = o.w;
  return q;
}

Rigid3d FromROS(const geometry_msgs::PoseWithCovariance &pose_msg) {
  Rigid3d pose;
  pose.translation() = FromROS(pose_msg.pose.position);
  pose.rotation()    = FromROS(pose_msg.pose.orientation);
  return pose;
}

geometry_msgs::PoseWithCovariance ToROS(const Rigid3d &pose) {
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

Time FromROS(const ros::Time &time) {
  return FromUniversal(time.sec * UniversalTimeScaleClock::f1 +
                       time.nsec / UniversalTimeScaleClock::f2);
}

ros::Time ToROS(const Time &time) {
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

Rigid3d FromProto(const proto::Rigid3d &r) {
  return {FromProto(r.translation()), FromProto(r.rotation())};
}
