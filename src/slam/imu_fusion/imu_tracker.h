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

#ifndef MSF_LOAM_VELODYNE_IMU_TRACKER_H
#define MSF_LOAM_VELODYNE_IMU_TRACKER_H

#include "Eigen/Geometry"
#include "common/time_def.h"

struct ImuData {
  Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};

// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
class ImuTracker {
 public:
  ImuTracker(double imu_gravity_time_constant, Time time);

  // Advances to the given 'time' and updates the orientation to reflect this.
  void Advance(Time time);

  // Updates from an IMU reading (in the IMU frame).
  void AddImuObservation(const ImuData& imu_data);

  // Query the current time.
  Time time() const { return time_; }

  // Query the current orientation estimate.
  Eigen::Quaterniond orientation() const { return orientation_; }

 private:
  const double imu_gravity_time_constant_;
  Time time_;
  Time last_linear_acceleration_time_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d gravity_vector_;
  Eigen::Vector3d imu_angular_velocity_;
};

#endif  // MSF_LOAM_VELODYNE_IMU_TRACKER_H
