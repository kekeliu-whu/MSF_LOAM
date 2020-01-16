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
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <cmath>
#include <string>
#include <vector>

#include "common/common.h"
#include "common/tic_toc.h"
#include "msg_conversion.h"
#include "slam/imu_fusion/imu_tracker.h"
#include "slam/local/laser_odometry.h"

DEFINE_bool(is_offline_mode, false, "Runtime mode: online or offline.");

DEFINE_string(bag_filename, "", "Bag file to read in offline mode.");

namespace {

enum PointLabel { P_UNKNOWN = 0, P_LESS_SHARP = 1, P_SHARP = 2, P_FLAT = -1 };

const double kScanPeriod = 0.1;  // 扫描周期
double g_min_range;              // 最小扫描距离
int g_scan_num;                  // 扫描线数

std::vector<float> g_cloud_curvatures(400000);    // 点的曲率
std::vector<int> g_cloud_sorted_indices(400000);  // 通过曲率对点排序
std::vector<bool> g_is_cloud_neighbor_picked(400000);  // 临近点是否已被选取
std::vector<int> g_cloud_labels(400000);  // 扫描线上点的类型

}  // namespace

template <typename PointT>
void RemoveClosePointsFromCloud(const pcl::PointCloud<PointT> &cloud_in,
                                pcl::PointCloud<PointT> &cloud_out,
                                double min_range) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }

  size_t j = 0;

  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    if (cloud_in.points[i].x * cloud_in.points[i].x +
            cloud_in.points[i].y * cloud_in.points[i].y +
            cloud_in.points[i].z * cloud_in.points[i].z <
        min_range * min_range)
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    j++;
  }
  if (j != cloud_in.points.size()) {
    cloud_out.points.resize(j);
  }

  cloud_out.height = 1;
  cloud_out.width = static_cast<uint32_t>(j);
  cloud_out.is_dense = true;
}

void HandleLaserCloudMessage(
    const sensor_msgs::PointCloud2ConstPtr &laser_cloud_msg,
    const std::shared_ptr<LaserOdometry> &laser_odometry_handler) {
  TicToc t_whole;
  TicToc t_prepare;
  std::vector<int> scan_start_indices(g_scan_num, 0);
  std::vector<int> scan_end_indices(g_scan_num, 0);

  pcl::PointCloud<pcl::PointXYZ> laser_cloud_in;
  pcl::fromROSMsg(*laser_cloud_msg, laser_cloud_in);

  // 删除非法点
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(laser_cloud_in, laser_cloud_in, indices);
  RemoveClosePointsFromCloud(laser_cloud_in, laser_cloud_in, g_min_range);

  int cloudSize = laser_cloud_in.points.size();
  double startOri =
      -atan2(laser_cloud_in.points[0].y, laser_cloud_in.points[0].x);
  double endOri = -atan2(laser_cloud_in.points[cloudSize - 1].y,
                         laser_cloud_in.points[cloudSize - 1].x) +
                  2 * M_PI;

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  int count = cloudSize;
  PointType point;
  std::vector<PointCloud> laserCloudScans(g_scan_num);
  for (int i = 0; i < cloudSize; i++) {
    point.x = laser_cloud_in.points[i].x;
    point.y = laser_cloud_in.points[i].y;
    point.z = laser_cloud_in.points[i].z;

    // 计算scan_id
    double angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) *
                   180 / M_PI;
    int scanID = 0;

    if (g_scan_num == 16) {
      scanID = int((angle + 15) / 2 + 0.5);
      if (scanID > (g_scan_num - 1) || scanID < 0) {
        count--;
        continue;
      }
    } else if (g_scan_num == 32) {
      scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
      if (scanID > (g_scan_num - 1) || scanID < 0) {
        count--;
        continue;
      }
    } else if (g_scan_num == 64) {
      if (angle >= -8.83)
        scanID = int((2 - angle) * 3.0 + 0.5);
      else
        scanID = g_scan_num / 2 + int((-8.83 - angle) * 2.0 + 0.5);

      // use [0 50]  > 50 remove outlies
      if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0) {
        count--;
        continue;
      }
    } else {
      LOG(FATAL) << "Wrong scan number:" << g_scan_num;
    }

    double ori = -atan2(point.y, point.x);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;
      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    // 密度的整数部分为scan_id，浮点部分为点在当前帧的时间偏移
    double relTime = (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + kScanPeriod * relTime;
    laserCloudScans[scanID].push_back(point);
  }
  LOG_IF(WARNING, cloudSize - count > 10)
      << "More than 10 invalid points: no matching scan id!!";

  cloudSize = count;
  LOG(INFO) << "[REG] Cloud size: " << cloudSize;

  PointCloudPtr laser_cloud(new PointCloud);
  for (int i = 0; i < g_scan_num; i++) {
    scan_start_indices[i] = laser_cloud->size() + 5;
    *laser_cloud += laserCloudScans[i];
    scan_end_indices[i] = laser_cloud->size() - 6;
  }

  LOG_STEP_TIME("REG", "Re-index scans", t_prepare.toc());

  /**
   * @brief 计算所有点的曲率
   *
   * dx(i) = x[i-5]+x[i-4]+...+x[i+5]-10*x[i]
   * ...
   * curv(i) = dx(i)^2 + dy(i)^2 + dz(i)^2
   */
  for (int i = 5; i < cloudSize - 5; i++) {
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

    g_cloud_curvatures[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    g_cloud_sorted_indices[i] = i;
    g_is_cloud_neighbor_picked[i] = false;
    g_cloud_labels[i] = P_UNKNOWN;
  }

  TicToc t_pts;

  PointCloudPtr cloud_corner_sharp(new PointCloud);       // sharp 点
  PointCloudPtr cloud_corner_less_sharp(new PointCloud);  // less sharp 点
  PointCloudPtr cloud_surf_flat(new PointCloud);          // flat 点
  PointCloudPtr cloud_surf_less_flat(new PointCloud);     // less flat 点

  double t_q_sort = 0;
  // 提取每帧扫描线中的特征点
  for (int i = 0; i < g_scan_num; i++) {
    if (scan_end_indices[i] - scan_start_indices[i] < 6) continue;
    PointCloudPtr surfPointsLessFlatScan(new PointCloud);
    // 将每条扫描线分成6片，对每片提取特征点
    for (int j = 0; j < 6; j++) {
      int sp = scan_start_indices[i] +
               (scan_end_indices[i] - scan_start_indices[i]) * j / 6;
      int ep = scan_start_indices[i] +
               (scan_end_indices[i] - scan_start_indices[i]) * (j + 1) / 6 - 1;

      TicToc t_tmp;
      // 对每片点云中的曲率排序
      std::sort(g_cloud_sorted_indices.begin() + sp,
                g_cloud_sorted_indices.begin() + ep + 1,
                [](int i, int j) -> bool {
                  return g_cloud_curvatures[i] < g_cloud_curvatures[j];
                });
      t_q_sort += t_tmp.toc();

      // 取曲率最高的前2个点为sharp点，前20个为less_sharp点（每次选取时标记周围的十一个点）
      int largest_picked_num = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = g_cloud_sorted_indices[k];

        if (!g_is_cloud_neighbor_picked[ind] && g_cloud_curvatures[ind] > 0.1) {
          largest_picked_num++;
          if (largest_picked_num <= 2) {
            g_cloud_labels[ind] = P_SHARP;
            cloud_corner_sharp->push_back(laser_cloud->points[ind]);
            cloud_corner_less_sharp->push_back(laser_cloud->points[ind]);
          } else if (largest_picked_num <= 20) {
            g_cloud_labels[ind] = P_LESS_SHARP;
            cloud_corner_less_sharp->push_back(laser_cloud->points[ind]);
          } else {
            break;
          }

          // 标记临近点
          g_is_cloud_neighbor_picked[ind] = true;
          for (int l = 1; l <= 5; l++) {
            auto vec = laser_cloud->points[ind + l].getVector3fMap() -
                       laser_cloud->points[ind + l - 1].getVector3fMap();
            if (vec.squaredNorm() > 0.05) break;
            g_is_cloud_neighbor_picked[ind + l] = true;
          }
          for (int l = -1; l >= -5; l--) {
            auto vec = laser_cloud->points[ind + l].getVector3fMap() -
                       laser_cloud->points[ind + l + 1].getVector3fMap();
            if (vec.squaredNorm() > 0.05) break;
            g_is_cloud_neighbor_picked[ind + l] = true;
          }
        }
      }

      // 取曲率最低的前4个点为flat点（每次选取时标记周围的十一个点）
      int smallest_picked_num = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = g_cloud_sorted_indices[k];

        if (!g_is_cloud_neighbor_picked[ind] && g_cloud_curvatures[ind] < 0.1) {
          g_cloud_labels[ind] = P_FLAT;
          cloud_surf_flat->push_back(laser_cloud->points[ind]);

          smallest_picked_num++;
          if (smallest_picked_num >= 4) {
            break;
          }

          // 标记临近点
          g_is_cloud_neighbor_picked[ind] = true;
          for (int l = 1; l <= 5; l++) {
            auto vec = laser_cloud->points[ind + l].getVector3fMap() -
                       laser_cloud->points[ind + l - 1].getVector3fMap();
            if (vec.squaredNorm() > 0.05) break;
            g_is_cloud_neighbor_picked[ind + l] = true;
          }
          for (int l = -1; l >= -5; l--) {
            auto vec = laser_cloud->points[ind + l].getVector3fMap() -
                       laser_cloud->points[ind + l + 1].getVector3fMap();
            if (vec.squaredNorm() > 0.05) break;
            g_is_cloud_neighbor_picked[ind + l] = true;
          }
        }
      }

      // 将flat点和未标记点都标记为less_flat点
      for (int k = sp; k <= ep; k++) {
        if (g_cloud_labels[k] == P_FLAT || g_cloud_labels[k] == P_UNKNOWN) {
          surfPointsLessFlatScan->push_back(laser_cloud->points[k]);
        }
      }
    }

    PointCloud surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    *cloud_surf_less_flat += surfPointsLessFlatScanDS;
  }
  LOG_STEP_TIME("REG", "Curvature sort", t_q_sort);
  LOG_STEP_TIME("REG", "Seperate points", t_pts.toc());

  TimestampedPointCloud scan;
  scan.timestamp = FromRos(laser_cloud_msg->header.stamp);
  scan.cloud_full_res = laser_cloud;
  scan.cloud_surf_less_flat = cloud_surf_less_flat;
  scan.cloud_surf_flat = cloud_surf_flat;
  scan.cloud_corner_less_sharp = cloud_corner_less_sharp;
  scan.cloud_corner_sharp = cloud_corner_sharp;
  laser_odometry_handler->AddLaserScan(scan);

  LOG_STEP_TIME("REG", "Scan registration", t_whole.toc());
  LOG_IF(WARNING, t_whole.toc() > 100)
      << "Scan registration process over 100ms";
}

void HandleImuMessage(
    const sensor_msgs::ImuConstPtr &imu_msg,
    const std::shared_ptr<LaserOdometry> &laser_odometry_handler) {
  laser_odometry_handler->AddImu({
      FromRos(imu_msg->header.stamp),
      {imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
       imu_msg->linear_acceleration.z},
      {imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
       imu_msg->angular_velocity.z}});
}

int main(int argc, char **argv) {
  // Set glog and gflags
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Set ROS node
  ros::init(argc, argv, "nsf_loam_node");
  ros::NodeHandle nh;

  LOG_IF(WARNING, !nh.param<int>("scan_line", g_scan_num, 16))
      << "Use default scan_line: 16";
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
        HandleLaserCloudMessage(m.instantiate<sensor_msgs::PointCloud2>(),
                                laser_odometry_handler);
      } else if (m.isType<sensor_msgs::Imu>()) {
        HandleImuMessage(m.instantiate<sensor_msgs::Imu>(),
                         laser_odometry_handler);
      }
    }
    bag.close();
  } else {
    LOG_IF(WARNING, !FLAGS_bag_filename.empty())
        << "Offline mode is on, so bag_filename will be ignored.";
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
        "/velodyne_points", 10,
        boost::bind(HandleLaserCloudMessage, _1,
                    boost::ref(laser_odometry_handler)));
    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>(
        "/imu", 10,
        boost::bind(HandleImuMessage, _1, boost::ref(laser_odometry_handler)));
    ros::spin();
  }

  return 0;
}
