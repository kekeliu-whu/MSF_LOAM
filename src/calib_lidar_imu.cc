#include <bits/stdint-uintn.h>
#include <fmt/core.h>
#include <algorithm>
#include <fstream>
#include <iterator>

#include "common/common.h"
#include "msg.pb.h"
#include "slam/imu_fusion/initial_ex_rotation.h"
#include "slam/imu_fusion/utility.h"
#include "slam/msg_conversion.h"

int main() {
  proto::PbData pb_data;
  std::ifstream ifs(kTrajectoryPbPath, std::ios_base::in | std::ios_base::binary);
  if (!ifs) {
    std::cout << ": File not found.  Creating a new file." << std::endl;
  } else if (!pb_data.ParseFromIstream(&ifs)) {
    std::cerr << "Failed to parse address book." << std::endl;
    return -1;
  }

  auto& imu_datas  = pb_data.imu_datas();
  auto& odom_datas = pb_data.imu_datas();

  //  std::cout << pb_data.imu_datas_size() << std::endl;
  //  std::cout << pb_data.odom_datas_size() << std::endl;

  InitialEXRotation initialExRotation;

  const int kStep = 3;
  for (int i = 500; i < pb_data.odom_datas_size() - kStep; i += kStep) {
    const auto& l1 = pb_data.odom_datas(i);
    auto l1_it     = std::lower_bound(imu_datas.begin(), imu_datas.end(), l1.timestamp(), [](const proto::ImuData& odom, uint64_t time) {
      return odom.timestamp() < time;
    });
    const auto& l2 = pb_data.odom_datas(i + kStep);
    auto l2_it     = std::lower_bound(imu_datas.begin(), imu_datas.end(), l2.timestamp(), [](const proto::ImuData& odom, uint64_t time) {
      return odom.timestamp() < time;
    });
    // it->PrintDebugString();
    std::cout << "add calib data: " << i << std::endl;
    //    std::cout << std::distance(l1_it, l2_it) << std::endl;

    Eigen::Quaterniond delta_q_imu = Eigen::Quaterniond(1, 0, 0, 0);
    for (auto it = l1_it; it != l2_it; ++it) {
      auto& av    = it->angular_velocity();
      auto dq     = Utility::deltaQ(Eigen::Vector3d(av.x(), av.y(), av.z()) * 0.01);
      delta_q_imu = delta_q_imu * dq;
    }

    // calib
    Eigen::Matrix3d result;
    bool calib_ok = initialExRotation.CalibrationExRotation(
        FromProto(l1.pose().rotation()).inverse() * FromProto(l2.pose().rotation()),
        delta_q_imu,
        result);

    if (calib_ok) {
      Eigen::AngleAxisd aa(result);
      fmt::print("frame_idx={} ,ok={}, axis=[{},{},{}], angle={}\n",
                 i, calib_ok, aa.axis().x(), aa.axis().y(), aa.axis().z(), aa.angle() * 180 / M_PI);
      break;
    }
  }
}
