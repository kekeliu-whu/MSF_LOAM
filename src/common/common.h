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

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define DISTORTION 0

const std::string kTrajectoryPbPath = "/tmp/trajectory.pbstream";

struct PointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
//   (float, time, time) // todo read point with field time
)
// clang-format on

using PointType          = pcl::PointXYZI;
using PointCloud         = pcl::PointCloud<PointType>;
using PointCloudPtr      = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

using PointTypeOriginal          = ::PointXYZIRT;
using PointCloudOriginal         = pcl::PointCloud<PointTypeOriginal>;
using PointCloudOriginalPtr      = PointCloudOriginal::Ptr;
using PointCloudOriginalConstPtr = PointCloudOriginal::ConstPtr;

using Vector3d    = Eigen::Vector3d;
using Matrix3d    = Eigen::Matrix3d;
using Quaterniond = Eigen::Quaterniond;
