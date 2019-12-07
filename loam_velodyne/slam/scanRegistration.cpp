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
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <cmath>
#include <string>
#include <vector>

#include "loam_velodyne/common/common.h"
#include "loam_velodyne/common/tic_toc.h"

using std::atan2;
using std::cos;
using std::sin;

namespace {

enum PointLabel { P_UNKNOWN = 0, P_LESS_SHARP = 1, P_SHARP = 2, P_FLAT = -1 };

const double kScanPeriod = 0.1;  // 扫描周期
int g_cloud_index_curr = 0;      // 当前扫描帧下标
double g_min_range;              // 最小扫描距离
int g_scan_num;                  // 扫描个数

std::vector<float> g_cloud_curvatures(400000);    // 点的曲率
std::vector<int> g_cloud_sorted_indices(400000);  // 通过曲率对点排序
std::vector<bool> g_is_cloud_neighbor_picked(400000);  // 临近点是否已被选取
std::vector<int> g_cloud_labels(400000);  // 扫描线上点的类型

ros::Publisher g_cloud_publisher;
ros::Publisher g_corner_cloud_publisher;
ros::Publisher g_corner_less_cloud_publisher;
ros::Publisher g_surf_cloud_publisher;
ros::Publisher g_surf_less_cloud_publisher;
ros::Publisher g_removed_cloud_publisher;

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

void HandleLaserCloudMsg(
    const sensor_msgs::PointCloud2ConstPtr &laser_cloud_msg) {
  g_cloud_index_curr++;

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
  float startOri =
      -atan2(laser_cloud_in.points[0].y, laser_cloud_in.points[0].x);
  float endOri = -atan2(laser_cloud_in.points[cloudSize - 1].y,
                        laser_cloud_in.points[cloudSize - 1].x) +
                 2 * M_PI;

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }
  // printf("end Ori %f\n", endOri);

  bool halfPassed = false;
  int count = cloudSize;
  PointType point;
  std::vector<pcl::PointCloud<PointType>> laserCloudScans(g_scan_num);
  for (int i = 0; i < cloudSize; i++) {
    point.x = laser_cloud_in.points[i].x;
    point.y = laser_cloud_in.points[i].y;
    point.z = laser_cloud_in.points[i].z;

    // 计算scan_id
    float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) *
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

    // TODO
    float ori = -atan2(point.y, point.x);
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
    float relTime = (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + kScanPeriod * relTime;
    laserCloudScans[scanID].push_back(point);
  }
  LOG_IF(WARNING, cloudSize - count > 10)
      << "More than 10 invalid points: no matching scan id!!";

  cloudSize = count;
  LOG(INFO) << "Cloud size: " << cloudSize;

  pcl::PointCloud<PointType>::Ptr laser_cloud(new pcl::PointCloud<PointType>());
  for (int i = 0; i < g_scan_num; i++) {
    scan_start_indices[i] = laser_cloud->size() + 5;
    *laser_cloud += laserCloudScans[i];
    scan_end_indices[i] = laser_cloud->size() - 6;
  }

  LOG_STEP_TIME("Re-index scans", t_prepare.toc());

  /**
   * @brief 计算所有点的曲率
   *
   * dx(i) = x[i-5]+x[i-4]+...+x[i+5]-10*x[i]
   * ...
   * curv(i) = dx(i)^2 + dy(i)^2 + dz(i)^2
   */
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffX = laser_cloud->points[i - 5].x + laser_cloud->points[i - 4].x +
                  laser_cloud->points[i - 3].x + laser_cloud->points[i - 2].x +
                  laser_cloud->points[i - 1].x - 10 * laser_cloud->points[i].x +
                  laser_cloud->points[i + 1].x + laser_cloud->points[i + 2].x +
                  laser_cloud->points[i + 3].x + laser_cloud->points[i + 4].x +
                  laser_cloud->points[i + 5].x;
    float diffY = laser_cloud->points[i - 5].y + laser_cloud->points[i - 4].y +
                  laser_cloud->points[i - 3].y + laser_cloud->points[i - 2].y +
                  laser_cloud->points[i - 1].y - 10 * laser_cloud->points[i].y +
                  laser_cloud->points[i + 1].y + laser_cloud->points[i + 2].y +
                  laser_cloud->points[i + 3].y + laser_cloud->points[i + 4].y +
                  laser_cloud->points[i + 5].y;
    float diffZ = laser_cloud->points[i - 5].z + laser_cloud->points[i - 4].z +
                  laser_cloud->points[i - 3].z + laser_cloud->points[i - 2].z +
                  laser_cloud->points[i - 1].z - 10 * laser_cloud->points[i].z +
                  laser_cloud->points[i + 1].z + laser_cloud->points[i + 2].z +
                  laser_cloud->points[i + 3].z + laser_cloud->points[i + 4].z +
                  laser_cloud->points[i + 5].z;

    g_cloud_curvatures[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    g_cloud_sorted_indices[i] = i;
    g_is_cloud_neighbor_picked[i] = false;
    g_cloud_labels[i] = P_UNKNOWN;
  }

  TicToc t_pts;

  pcl::PointCloud<PointType> cloud_corner_sharp;       // sharp 点
  pcl::PointCloud<PointType> cloud_corner_less_sharp;  // less sharp 点
  pcl::PointCloud<PointType> cloud_surf_flat;          // flat 点
  pcl::PointCloud<PointType> cloud_surf_less_flat;     // less flat 点

  float t_q_sort = 0;
  // 提取每帧扫描线中的特征点
  for (int i = 0; i < g_scan_num; i++) {
    if (scan_end_indices[i] - scan_start_indices[i] < 6) continue;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(
        new pcl::PointCloud<PointType>);
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
            cloud_corner_sharp.push_back(laser_cloud->points[ind]);
            cloud_corner_less_sharp.push_back(laser_cloud->points[ind]);
          } else if (largest_picked_num <= 20) {
            g_cloud_labels[ind] = P_LESS_SHARP;
            cloud_corner_less_sharp.push_back(laser_cloud->points[ind]);
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
          cloud_surf_flat.push_back(laser_cloud->points[ind]);

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

    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    cloud_surf_less_flat += surfPointsLessFlatScanDS;
  }
  LOG_STEP_TIME("Curvature sort", t_q_sort);
  LOG_STEP_TIME("Seperate points", t_pts.toc());

  sensor_msgs::PointCloud2 laser_cloud_out_msg;
  pcl::toROSMsg(*laser_cloud, laser_cloud_out_msg);
  laser_cloud_out_msg.header.stamp = laser_cloud_msg->header.stamp;
  laser_cloud_out_msg.header.frame_id = "/aft_mapped";
  g_cloud_publisher.publish(laser_cloud_out_msg);

  sensor_msgs::PointCloud2 cloud_corner_sharp_msg;
  pcl::toROSMsg(cloud_corner_sharp, cloud_corner_sharp_msg);
  cloud_corner_sharp_msg.header.stamp = laser_cloud_msg->header.stamp;
  cloud_corner_sharp_msg.header.frame_id = "/aft_mapped";
  g_corner_cloud_publisher.publish(cloud_corner_sharp_msg);

  sensor_msgs::PointCloud2 cloud_corner_less_sharp_msg;
  pcl::toROSMsg(cloud_corner_less_sharp, cloud_corner_less_sharp_msg);
  cloud_corner_less_sharp_msg.header.stamp = laser_cloud_msg->header.stamp;
  cloud_corner_less_sharp_msg.header.frame_id = "/aft_mapped";
  g_corner_less_cloud_publisher.publish(cloud_corner_less_sharp_msg);

  sensor_msgs::PointCloud2 cloud_surf_flat_msg;
  pcl::toROSMsg(cloud_surf_flat, cloud_surf_flat_msg);
  cloud_surf_flat_msg.header.stamp = laser_cloud_msg->header.stamp;
  cloud_surf_flat_msg.header.frame_id = "/aft_mapped";
  g_surf_cloud_publisher.publish(cloud_surf_flat_msg);

  sensor_msgs::PointCloud2 cloud_surf_less_flat_msg;
  pcl::toROSMsg(cloud_surf_less_flat, cloud_surf_less_flat_msg);
  cloud_surf_less_flat_msg.header.stamp = laser_cloud_msg->header.stamp;
  cloud_surf_less_flat_msg.header.frame_id = "/aft_mapped";
  g_surf_less_cloud_publisher.publish(cloud_surf_less_flat_msg);

  LOG_STEP_TIME("Scan registration", t_whole.toc());
  LOG_IF(WARNING, t_whole.toc() > 100)
      << "Scan registration process over 100ms";
}

int main(int argc, char **argv) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  // 获取ROS参数
  CHECK(nh.param<int>("scan_line", g_scan_num, 16))
      << "Get param 'scan_line' failed!!";
  LOG_IF(WARNING, !nh.param<double>("minimum_range", g_min_range, 0.1))
      << "Use default minimum range: 0.1";
  CHECK(g_scan_num == 16 || g_scan_num == 32 || g_scan_num == 64)
      << "only support velodyne with 16, 32 or 64 scan line!";

  // 注册订阅器和发布器
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_points", 10, HandleLaserCloudMsg);
  g_cloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);
  g_corner_cloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
  g_corner_less_cloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);
  g_surf_cloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
  g_surf_less_cloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);
  g_removed_cloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

  ros::spin();

  return 0;
}
