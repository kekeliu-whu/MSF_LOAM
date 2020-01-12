//
// Created by whu on 1/11/20.
//

#include "slam/gps_fusion/gps_fusion.h"
#include "slam/gps_fusion/gps_factor.h"

GpsFusion::GpsFusion() {}

GpsFusion::~GpsFusion() {}

void GpsFusion::AddFixedPoint(const Time& time, const Eigen::Vector3d& t) {}

void GpsFusion::AddLocalPose(const Time& time, const Rigid3d& pose) {}

void GpsFusion::Optimize() {}
