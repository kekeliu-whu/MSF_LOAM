// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gflags/gflags.h>
#include <nav_msgs/Odometry.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>

#include "common/common.h"
#include "common/tic_toc.h"
#include "slam/local/laser_odometry.h"
#include "slam/msg_conversion.h"

DEFINE_bool(is_offline_mode, false, "Runtime mode: online or offline.");

DEFINE_string(bag_filename, "", "Bag file to read in offline mode.");

namespace {

/**
 * 特征点提取步骤如下：
 * 1. 将每条扫描线分为6个分片，每个分片约180个点；
 * 2. 将每个分片的所有点按照曲率从大到小排序；
 * 3.
 * 标记曲率最高的2个点为P_SHARP，最高的20个点为P_LESS_SHARP，邻域标为P_LESS_SHARP；
 * 4. 标记曲率最低的4个点为P_FLAT；
 * 5. 标记所有的P_FLAT点和P_UNKNOWN点为P_LESS_FLAT点，并做降采样。
 */
enum class PointLabel {
  // 未标记类型的点
  P_UNKNOWN,
  // 边角点
  P_SHARP,
  // 次边角点
  P_LESS_SHARP,
  // 平面点
  P_FLAT,
};

const int kDefaultScanNum = 16;
const double kScanPeriod  = 0.1;  // 扫描周期
double g_min_range;               // 最小扫描距离
int g_scan_num;                   // 扫描线数
std::uint64_t g_imu_msgs_num = 0;
sensor_msgs::PointCloud2ConstPtr g_prev_laser_cloud_msgs;

template <typename PointT>
void RemoveInvalidPointsFromCloud(const pcl::PointCloud<PointT> &cloud_in,
                                  pcl::PointCloud<PointT> &cloud_out,
                                  double min_range) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.resize(cloud_in.size());
  }

  size_t j = 0;

  for (size_t i = 0; i < cloud_in.size(); ++i) {
    if (cloud_in[i].getVector3fMap().norm() < min_range ||
        !std::isfinite(cloud_in[i].x) ||
        !std::isfinite(cloud_in[i].y) ||
        !std::isfinite(cloud_in[i].z)) continue;
    cloud_out[j] = cloud_in[i];
    j++;
  }
  if (j != cloud_in.size()) {
    cloud_out.resize(j);
  }

  cloud_out.height   = 1;
  cloud_out.width    = static_cast<uint32_t>(j);
  cloud_out.is_dense = true;
}

template <typename pointT>
void VoxelGridWrapper(const pcl::PointCloud<pointT> &cloud_in, pcl::PointCloud<pointT> &cloud_out) {
  PointCloudPtr cloud_in_without_rt{new PointCloud};
  pcl::copyPointCloud(cloud_in, *cloud_in_without_rt);

  PointCloud pc_out_without_rt;
  pcl::VoxelGrid<PointType> downSizeFilter;
  downSizeFilter.setInputCloud(cloud_in_without_rt);
  downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
  downSizeFilter.filter(pc_out_without_rt);
  auto indices = downSizeFilter.getIndices();

  pcl::copyPointCloud(cloud_in, *indices, cloud_out);
}

void ComputeRelaTimeForEachPoint(const PointCloudOriginal &laser_cloud_in, std::vector<PointCloudOriginal> &cloud_with_relative_time) {
  // lack of timestamp offset of first point/package.
  // todo will cause if timestamp offset of first point/package is not tiny.
  double start_ori = -atan2(laser_cloud_in.front().y, laser_cloud_in.front().x);

  std::vector<double> last_relative_angles(cloud_with_relative_time.size(), -1);
  for (int i = 0; i < laser_cloud_in.size(); i++) {
    PointTypeOriginal point = laser_cloud_in[i];
    CHECK_LT(point.ring, cloud_with_relative_time.size());

    // ring point are not ordered in CCW
    double ori = -atan2(point.y, point.x);

    // clamp angle to [0, 2*pi)
    double relative_angle = std::fmod(ori - start_ori + 2 * M_PI, 2 * M_PI);
    // LOG(INFO) << point.ring << " " << relative_angle;

    if (relative_angle < last_relative_angles[point.ring]) {
      // relative_angle might be over 2*pi
      relative_angle += 2 * M_PI;
    }
    last_relative_angles[point.ring] = relative_angle;

    double rela_time = relative_angle / (2 * M_PI) * kScanPeriod;
    point.time       = static_cast<float>(rela_time);
    cloud_with_relative_time[point.ring].push_back(point);
  }
}

}  // namespace

void RealHandleLaserCloudMessage(
    const sensor_msgs::PointCloud2ConstPtr &laser_cloud_msg,
    const std::shared_ptr<LaserOdometry> &laser_odometry_handler) {
  TicToc t_whole;
  TicToc t_prepare;
  std::vector<int> scan_start_indices(g_scan_num, 0);
  std::vector<int> scan_end_indices(g_scan_num, 0);

  pcl::PointCloud<PointTypeOriginal> laser_cloud_in;
  pcl::fromROSMsg(*laser_cloud_msg, laser_cloud_in);

  // 1. remove invalid points
  RemoveInvalidPointsFromCloud(laser_cloud_in, laser_cloud_in, g_min_range);

  // 2. compute point rel_time
  int cloudSize = laser_cloud_in.size();
  LOG(INFO) << "[REG] Valid Cloud size: " << cloudSize;

  std::vector<PointCloudOriginal> laserCloudScans(g_scan_num);
  ComputeRelaTimeForEachPoint(laser_cloud_in, laserCloudScans);

  PointCloudOriginalPtr laser_cloud(new PointCloudOriginal);
  for (int i = 0; i < g_scan_num; i++) {
    scan_start_indices[i] = laser_cloud->size() + 5;
    *laser_cloud += laserCloudScans[i];
    scan_end_indices[i] = static_cast<int>(laser_cloud->size()) - 6;
  }

  LOG_STEP_TIME("REG", "Compute relative time for scan points", t_prepare.toc());

  std::size_t _N = laser_cloud->size();
  CHECK_GT(_N, 0);
  std::vector<float> cloud_curvatures(_N);         // 点的曲率
  std::vector<int> cloud_sorted_indices(_N);       // 通过曲率对点排序
  std::vector<bool> cloud_is_neighbor_picked(_N);  // 临近点是否已被选取
  std::vector<PointLabel> cloud_labels(_N);        // 扫描线上点的类型

  /**
   * @brief 计算所有点的曲率
   *
   * dx(i) = x[i-5]+x[i-4]+...+x[i+5]-10*x[i]
   * ...
   * curv(i) = dx(i)^2 + dy(i)^2 + dz(i)^2
   */
  for (int i = 5; i < laser_cloud->size() - 5; i++) {
    double diffX = laser_cloud->points[i - 5].x + laser_cloud->points[i - 4].x +
                   laser_cloud->points[i - 3].x + laser_cloud->points[i - 2].x +
                   laser_cloud->points[i - 1].x -
                   10 * laser_cloud->points[i].x +
                   laser_cloud->points[i + 1].x + laser_cloud->points[i + 2].x +
                   laser_cloud->points[i + 3].x + laser_cloud->points[i + 4].x +
                   laser_cloud->points[i + 5].x;
    double diffY = laser_cloud->points[i - 5].y + laser_cloud->points[i - 4].y +
                   laser_cloud->points[i - 3].y + laser_cloud->points[i - 2].y +
                   laser_cloud->points[i - 1].y -
                   10 * laser_cloud->points[i].y +
                   laser_cloud->points[i + 1].y + laser_cloud->points[i + 2].y +
                   laser_cloud->points[i + 3].y + laser_cloud->points[i + 4].y +
                   laser_cloud->points[i + 5].y;
    double diffZ = laser_cloud->points[i - 5].z + laser_cloud->points[i - 4].z +
                   laser_cloud->points[i - 3].z + laser_cloud->points[i - 2].z +
                   laser_cloud->points[i - 1].z -
                   10 * laser_cloud->points[i].z +
                   laser_cloud->points[i + 1].z + laser_cloud->points[i + 2].z +
                   laser_cloud->points[i + 3].z + laser_cloud->points[i + 4].z +
                   laser_cloud->points[i + 5].z;

    cloud_curvatures[i]         = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloud_sorted_indices[i]     = i;
    cloud_is_neighbor_picked[i] = false;
    cloud_labels[i]             = PointLabel::P_UNKNOWN;
  }

  TicToc t_pts;

  PointCloudOriginalPtr cloud_corner_sharp(new PointCloudOriginal);       // sharp 点
  PointCloudOriginalPtr cloud_corner_less_sharp(new PointCloudOriginal);  // less sharp 点
  PointCloudOriginalPtr cloud_surf_flat(new PointCloudOriginal);          // flat 点
  PointCloudOriginalPtr cloud_surf_less_flat(new PointCloudOriginal);     // less flat 点

  double t_q_sort = 0;
  // 提取每帧扫描线中的特征点
  for (int i = 0; i < g_scan_num; i++) {
    if (scan_end_indices[i] - scan_start_indices[i] < 6) continue;
    PointCloudOriginalPtr surfPointsLessFlatScan(new PointCloudOriginal);
    // 将每条扫描线分成6片，对每片提取特征点
    for (int j = 0; j < 6; j++) {
      int sp = scan_start_indices[i] +
               (scan_end_indices[i] - scan_start_indices[i]) * j / 6;
      int ep = scan_start_indices[i] +
               (scan_end_indices[i] - scan_start_indices[i]) * (j + 1) / 6 - 1;

      TicToc t_tmp;
      // 对每片点云中的曲率排序
      std::sort(cloud_sorted_indices.begin() + sp,
                cloud_sorted_indices.begin() + ep + 1,
                [&cloud_curvatures](int i, int j) -> bool {
                  return cloud_curvatures[i] < cloud_curvatures[j];
                });
      t_q_sort += t_tmp.toc();

      // 取曲率最高的前2个点为sharp点，前20个为less_sharp点
      int largest_picked_num = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloud_sorted_indices[k];

        if (!cloud_is_neighbor_picked[ind] && cloud_curvatures[ind] > 0.1) {
          largest_picked_num++;
          if (largest_picked_num <= 2) {
            cloud_labels[ind] = PointLabel::P_SHARP;
            cloud_corner_sharp->push_back(laser_cloud->points[ind]);
            cloud_corner_less_sharp->push_back(laser_cloud->points[ind]);
          } else if (largest_picked_num <= 20) {
            cloud_labels[ind] = PointLabel::P_LESS_SHARP;
            cloud_corner_less_sharp->push_back(laser_cloud->points[ind]);
          } else {
            break;
          }

          // 标记临近点
          cloud_is_neighbor_picked[ind] = true;
          for (int l = 1; l <= 5; l++) {
            auto vec = laser_cloud->points[ind + l].getVector3fMap() -
                       laser_cloud->points[ind + l - 1].getVector3fMap();
            if (vec.squaredNorm() > 0.05) break;
            cloud_is_neighbor_picked[ind + l] = true;
            cloud_labels[ind + l]             = PointLabel::P_LESS_SHARP;
          }
          for (int l = -1; l >= -5; l--) {
            auto vec = laser_cloud->points[ind + l].getVector3fMap() -
                       laser_cloud->points[ind + l + 1].getVector3fMap();
            if (vec.squaredNorm() > 0.05) break;
            cloud_is_neighbor_picked[ind + l] = true;
            cloud_labels[ind + l]             = PointLabel::P_LESS_SHARP;
          }
        }
      }

      // 取曲率最低的前4个点为flat点
      int smallest_picked_num = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloud_sorted_indices[k];

        if (!cloud_is_neighbor_picked[ind] && cloud_curvatures[ind] < 0.1) {
          cloud_labels[ind] = PointLabel::P_FLAT;
          cloud_surf_flat->push_back(laser_cloud->points[ind]);

          smallest_picked_num++;
          if (smallest_picked_num >= 4) {
            break;
          }

          // 标记临近点
          cloud_is_neighbor_picked[ind] = true;
          for (int l = 1; l <= 5; l++) {
            auto vec = laser_cloud->points[ind + l].getVector3fMap() -
                       laser_cloud->points[ind + l - 1].getVector3fMap();
            if (vec.squaredNorm() > 0.05) break;
            cloud_is_neighbor_picked[ind + l] = true;
          }
          for (int l = -1; l >= -5; l--) {
            auto vec = laser_cloud->points[ind + l].getVector3fMap() -
                       laser_cloud->points[ind + l + 1].getVector3fMap();
            if (vec.squaredNorm() > 0.05) break;
            cloud_is_neighbor_picked[ind + l] = true;
          }
        }
      }

      // 将flat点和未标记点都标记为less_flat点
      for (int k = sp; k <= ep; k++) {
        if (cloud_labels[k] == PointLabel::P_FLAT ||
            cloud_labels[k] == PointLabel::P_UNKNOWN) {
          surfPointsLessFlatScan->push_back(laser_cloud->points[k]);
        }
      }
    }

    PointCloudOriginal surfPointsLessFlatScanDS;
    VoxelGridWrapper(*surfPointsLessFlatScan, surfPointsLessFlatScanDS);

    *cloud_surf_less_flat += surfPointsLessFlatScanDS;
  }
  LOG_STEP_TIME("REG", "Curvature sort", t_q_sort);
  LOG_STEP_TIME("REG", "Separate points into flat point and corner point", t_pts.toc());

  TimestampedPointCloud<PointTypeOriginal> scan;
  scan.time                    = FromRos(laser_cloud_msg->header.stamp);
  scan.cloud_full_res          = laser_cloud;
  scan.cloud_surf_less_flat    = cloud_surf_less_flat;
  scan.cloud_surf_flat         = cloud_surf_flat;
  scan.cloud_corner_less_sharp = cloud_corner_less_sharp;
  scan.cloud_corner_sharp      = cloud_corner_sharp;
  laser_odometry_handler->AddLaserScan(scan);

  LOG_STEP_TIME("REG", "Scan registration", t_whole.toc());
  LOG_IF(WARNING, t_whole.toc() > 100)
      << "Scan registration process over 100ms";
}

void TryHandleLaserCloudMessageWithImuIntegrated(
    const sensor_msgs::PointCloud2ConstPtr &laser_cloud_msg,
    const std::shared_ptr<LaserOdometry> &laser_odometry_handler) {
  // todo remove magic number
  if (g_imu_msgs_num <= 200) {
    LOG(WARNING) << "Waiting for imu data...";
    return;
  }
  if (g_prev_laser_cloud_msgs) {
    // delay to handle laser message for retrieving imu data
    RealHandleLaserCloudMessage(g_prev_laser_cloud_msgs, laser_odometry_handler);
  }
  g_prev_laser_cloud_msgs = laser_cloud_msg;
}

void HandleImuMessage(
    const sensor_msgs::ImuConstPtr &imu_msg,
    const std::shared_ptr<LaserOdometry> &laser_odometry_handler) {
  g_imu_msgs_num++;
  ImuData imu_data;
  imu_data.time = FromRos(imu_msg->header.stamp);
  imu_data.linear_acceleration << imu_msg->linear_acceleration.x,
      imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
  imu_data.angular_velocity << imu_msg->angular_velocity.x,
      imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
  laser_odometry_handler->AddImu(imu_data);
}

void HandleOdomMessage(
    const nav_msgs::OdometryConstPtr &odom_msg,
    const std::shared_ptr<LaserOdometry> &laser_odometry_handler) {
  OdometryData odom_data;
  odom_data.timestamp = FromRos(odom_msg->header.stamp);
  odom_data.odom      = FromRos(odom_msg->pose);
  odom_data.error     = 0;
  laser_odometry_handler->AddOdom(odom_data);
}

int main(int argc, char **argv) {
  // Set glog and gflags
  FLAGS_alsologtostderr  = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Set ROS node
  ros::init(argc, argv, "nsf_loam_node");
  ros::NodeHandle nh;

  LOG_IF(WARNING, !nh.param<int>("scan_line", g_scan_num, kDefaultScanNum))
      << "Use default scan_line: " << kDefaultScanNum;
  LOG_IF(WARNING, !nh.param<double>("minimum_range", g_min_range, 0.3))
      << "Use default minimum_range: 0.3";
  CHECK(g_scan_num == 16 || g_scan_num == 32 || g_scan_num == 64)
      << "only support velodyne with 16, 32 or 64 scan line!";

  auto laser_odometry_handler =
      std::make_shared<LaserOdometry>(FLAGS_is_offline_mode);

  if (FLAGS_is_offline_mode) {
    CHECK(!FLAGS_bag_filename.empty());
    LOG(INFO) << "Using offline mode ...";
    rosbag::Bag bag;
    bag.open(FLAGS_bag_filename);
    LOG(INFO) << "Reading bag file " << FLAGS_bag_filename << " ...";
    for (auto &m : rosbag::View(bag)) {
      if (m.isType<sensor_msgs::PointCloud2>()) {
        TryHandleLaserCloudMessageWithImuIntegrated(m.instantiate<sensor_msgs::PointCloud2>(),
                                                    laser_odometry_handler);
      } else if (m.isType<sensor_msgs::Imu>()) {
        HandleImuMessage(m.instantiate<sensor_msgs::Imu>(),
                         laser_odometry_handler);
      } else if (m.isType<nav_msgs::Odometry>()) {
        HandleOdomMessage(m.instantiate<nav_msgs::Odometry>(),
                          laser_odometry_handler);
      }
    }
    bag.close();
  } else {
    LOG_IF(WARNING, !FLAGS_bag_filename.empty())
        << "Online mode is enabled, so the parameter 'bag_filename' will be ignored.";
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
        "/velodyne_points", 100,
        boost::bind(TryHandleLaserCloudMessageWithImuIntegrated, _1,
                    boost::ref(laser_odometry_handler)));
    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>(
        "/imu", 1000,
        boost::bind(HandleImuMessage, _1, boost::ref(laser_odometry_handler)));
    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>(
        "/odometry_gt", 100,
        boost::bind(HandleOdomMessage, _1, boost::ref(laser_odometry_handler)));
    ros::spin();
  }

  return 0;
}
