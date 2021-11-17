//
// Created by kekeliu on 12/17/19.
//

#include <pcl/kdtree/kdtree_flann.h>

#include "common/tic_toc.h"
#include "lidar_factor.h"
#include "mapping_scan_matcher.h"
#include "slam/imu_fusion/pose_local_parameterization.h"

namespace {
constexpr int kOptimalNum = 2;
}

bool MappingScanMatcher::MatchScan2Map(const TimestampedPointCloud<PointType> &cloud_map,
                                       const TimestampedPointCloud<PointType> &scan_curr,
                                       const std::shared_ptr<IntegrationBase> &preintegration,
                                       Rigid3d *pose_estimate_map_scan2world) {
  TicToc t_opt;
  TicToc t_tree;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(
      new pcl::KdTreeFLANN<PointType>());
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(
      new pcl::KdTreeFLANN<PointType>());

  kdtreeCornerFromMap->setInputCloud(cloud_map.cloud_corner_less_sharp);
  kdtreeSurfFromMap->setInputCloud(cloud_map.cloud_surf_less_flat);
  LOG_STEP_TIME("MAP", "build tree", t_tree.toc());

  for (int iterCount = 0; iterCount < kOptimalNum; iterCount++) {
    // ceres::LossFunction *loss_function = NULL;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *se3_parameterization =
        new PoseLocalParameterization();
    ceres::Problem::Options problem_options;

    ceres::Problem problem(problem_options);
    auto pose_params = pose_estimate_map_scan2world->ToVector7();
    problem.AddParameterBlock(pose_params.data(), 7, se3_parameterization);

    TicToc t_data;
    int corner_num = 0;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    PointType pointOri, pointSel;

    for (size_t i = 0; i < scan_curr.cloud_corner_less_sharp->size(); i++) {
      pointOri = scan_curr.cloud_corner_less_sharp->points[i];
      pointSel = *pose_estimate_map_scan2world * pointOri;
      kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                          pointSearchSqDis);

      if (pointSearchSqDis[4] < 1.0) {
        TicToc t;
        Eigen::Matrix<double, 3, 5> matA0;
        for (int j = 0; j < 5; j++) {
          matA0.col(j) =
              cloud_map.cloud_corner_less_sharp->points[pointSearchInd[j]]
                  .getVector3fMap()
                  .cast<double>();
        }
        Eigen::Vector3d center = matA0.rowwise().mean();
        matA0                  = matA0.colwise() - center;
        Eigen::Matrix3d covMat = matA0 * matA0.transpose();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

        // if is indeed line feature
        // note Eigen library sort eigenvalues in increasing order
        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
          const Eigen::Vector3d &point_on_line = center;
          Eigen::Vector3d point_a, point_b;
          point_a = 0.1 * unit_direction + point_on_line;
          point_b = -0.1 * unit_direction + point_on_line;

          ceres::CostFunction *cost_function =
              new LidarEdgeFactorSE3(curr_point, point_a, (point_a - point_b).normalized());
          problem.AddResidualBlock(
              cost_function, loss_function,
              pose_params.data());
          corner_num++;
        }
      }
    }

    int surf_num = 0;
    for (size_t i = 0; i < scan_curr.cloud_surf_less_flat->size(); i++) {
      pointOri = scan_curr.cloud_surf_less_flat->points[i];
      pointSel = *pose_estimate_map_scan2world * pointOri;
      kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                        pointSearchSqDis);

      if (pointSearchSqDis[4] < 1.0) {
        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 =
            -1 * Eigen::Matrix<double, 5, 1>::Ones();
        for (int j = 0; j < 5; j++) {
          matA0.row(j) =
              cloud_map.cloud_surf_less_flat->points[pointSearchInd[j]]
                  .getVector3fMap()
                  .cast<double>();
        }
        // 平面到原点的垂直向量
        // 原理：平面ax+by+cz+D=0的法向量是(a,b,c)
        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
        norm.normalize();
        Eigen::Vector3d center = matA0.colwise().mean();

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
          if (std::abs(norm.dot(matA0.row(j).transpose() - center)) > 0.2) {
            planeValid = false;
            break;
          }
        }
        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
        if (planeValid) {
          ceres::CostFunction *cost_function =
              new LidarPlaneFactorSE3(curr_point, center, norm);
          problem.AddResidualBlock(
              cost_function, loss_function,
              pose_params.data());
          surf_num++;
        }
      }
    }

    LOG_STEP_TIME("MAP", "Data association", t_data.toc());

    TicToc t_solver;
    ceres::Solver::Options options;
    options.max_num_iterations           = 6;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    if (iterCount == kOptimalNum - 1) {
      this->RefineByRejectOutliersWithThreshold(problem, 1);
    }
    ceres::Solve(options, &problem, &summary);
    LOG_STEP_TIME("MAP", "Solver time", t_solver.toc());

    // attention: update by optimized pose_params(vector7)
    *pose_estimate_map_scan2world = pose_params;
  }
  LOG_STEP_TIME("MAP", "Optimization twice", t_opt.toc());

  return true;
}
