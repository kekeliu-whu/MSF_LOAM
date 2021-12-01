/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MSF_LOAM_VELODYNE_HYBRID_GRID_H
#define MSF_LOAM_VELODYNE_HYBRID_GRID_H

#include <pcl/filters/filter.h>

#include <common/rigid_transform.h>
#include "common/common.h"

class HybridGridImpl;

class HybridGrid {
 public:
  explicit HybridGrid(const float& resolution);
  virtual ~HybridGrid() = default;

  PointCloudPtr GetSurroundedCloud(const PointCloudConstPtr& scan,
                                   const Rigid3d& pose);

  void InsertScan(const PointCloudPtr& scan, pcl::Filter<PointType>& filter);

 private:
  std::shared_ptr<HybridGridImpl> hybrid_grid_;
};

#endif  // MSF_LOAM_VELODYNE_HYBRID_GRID_H
