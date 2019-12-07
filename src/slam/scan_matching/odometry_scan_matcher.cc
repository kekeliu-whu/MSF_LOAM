//
// Created by whu on 12/7/19.
//

#include <ceres/ceres.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "common/common.h"
#include "common/tic_toc.h"
#include "lidar_factor.h"
#include "odometry_scan_matcher.h"

namespace {

constexpr double kScanPeriod = 0.1;
constexpr double kDistanceSqThreshold = 25;
constexpr double kNearByScan = 2.5;

// undistort lidar point
void TransformToStart(const PointType &pi, PointType &po,
                      const Rigid3d &transform_curr2last) {
  // interpolation ratio
  double s;
  if (DISTORTION)
    s = (pi.intensity - int(pi.intensity)) / kScanPeriod;
  else
    s = 1.0;
  // s = 1;
  Eigen::Quaterniond q_point_last =
      Eigen::Quaterniond::Identity().slerp(s, transform_curr2last.rotation());
  Eigen::Vector3d t_point_last = s * transform_curr2last.translation();
  Eigen::Vector3d point(pi.x, pi.y, pi.z);
  Eigen::Vector3d un_point = q_point_last * point + t_point_last;

  po.x = un_point.x();
  po.y = un_point.y();
  po.z = un_point.z();
  po.intensity = pi.intensity;
}

}  // namespace

bool OdometryScanMatcher::Match(const TimestampedPointCloud &cloud_last,
                                const TimestampedPointCloud &cloud_curr,
                                Rigid3d *pose_estimate_curr2last) {
  Eigen::Quaterniond &r_curr2last = pose_estimate_curr2last->rotation();
  Eigen::Vector3d &t_curr2last = pose_estimate_curr2last->translation();

  pcl::PointCloud<PointType>::Ptr cloud_corner_sharp =
      cloud_curr.cloud_corner_sharp;
  pcl::PointCloud<PointType>::Ptr cloud_corner_less_sharp =
      cloud_curr.cloud_corner_less_sharp;
  pcl::PointCloud<PointType>::Ptr cloud_surf_flat = cloud_curr.cloud_surf_flat;
  pcl::PointCloud<PointType>::Ptr cloud_surf_less_flat =
      cloud_curr.cloud_surf_less_flat;

  pcl::PointCloud<PointType>::Ptr cloud_corner_last =
      cloud_last.cloud_corner_less_sharp;
  pcl::PointCloud<PointType>::Ptr cloud_surf_last =
      cloud_last.cloud_surf_less_flat;

  // less sharp 点构造的 kdtree
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast(
      new pcl::KdTreeFLANN<PointType>());
  kdtreeCornerLast->setInputCloud(cloud_corner_last);
  // less flat 点构造的 kdtree
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast(
      new pcl::KdTreeFLANN<PointType>());
  kdtreeSurfLast->setInputCloud(cloud_surf_last);

  TicToc t_opt;

  for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
    int corner_correspondence = 0, plane_correspondence = 0;

    // ceres::LossFunction *loss_function = NULL;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization =
        new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;

    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(r_curr2last.coeffs().data(), 4,
                              q_parameterization);
    problem.AddParameterBlock(t_curr2last.data(), 3);

    PointType pointSel;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    TicToc t_data;
    // find correspondence for corner features
    for (size_t i = 0; i < cloud_corner_sharp->size(); ++i) {
      TransformToStart(cloud_corner_sharp->points[i], pointSel,
                       *pose_estimate_curr2last);
      kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd,
                                       pointSearchSqDis);

      int closestPointInd = -1, minPointInd2 = -1;
      if (pointSearchSqDis[0] < kDistanceSqThreshold) {
        closestPointInd = pointSearchInd[0];
        int closestPointScanID =
            int(cloud_corner_last->points[closestPointInd].intensity);

        double minPointSqDis2 = kDistanceSqThreshold;
        // search in the direction of increasing scan line
        for (int j = closestPointInd + 1; j < (int)cloud_corner_last->size();
             ++j) {
          // if in the same scan line, continue
          if (int(cloud_corner_last->points[j].intensity) <= closestPointScanID)
            continue;

          // if not in nearby scans, end the loop
          if (int(cloud_corner_last->points[j].intensity) >
              (closestPointScanID + kNearByScan))
            break;

          double pointSqDis =
              (cloud_corner_last->points[j].x - pointSel.x) *
                  (cloud_corner_last->points[j].x - pointSel.x) +
              (cloud_corner_last->points[j].y - pointSel.y) *
                  (cloud_corner_last->points[j].y - pointSel.y) +
              (cloud_corner_last->points[j].z - pointSel.z) *
                  (cloud_corner_last->points[j].z - pointSel.z);

          if (pointSqDis < minPointSqDis2) {
            // find nearer point
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
          }
        }

        // search in the direction of decreasing scan line
        for (int j = closestPointInd - 1; j >= 0; --j) {
          // if in the same scan line, continue
          if (int(cloud_corner_last->points[j].intensity) >= closestPointScanID)
            continue;

          // if not in nearby scans, end the loop
          if (int(cloud_corner_last->points[j].intensity) <
              (closestPointScanID - kNearByScan))
            break;

          double pointSqDis =
              (cloud_corner_last->points[j].x - pointSel.x) *
                  (cloud_corner_last->points[j].x - pointSel.x) +
              (cloud_corner_last->points[j].y - pointSel.y) *
                  (cloud_corner_last->points[j].y - pointSel.y) +
              (cloud_corner_last->points[j].z - pointSel.z) *
                  (cloud_corner_last->points[j].z - pointSel.z);

          if (pointSqDis < minPointSqDis2) {
            // find nearer point
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
          }
        }
      }
      // both closestPointInd and minPointInd2 is valid
      if (minPointInd2 >= 0) {
        Eigen::Vector3d curr_point(cloud_corner_sharp->points[i].x,
                                   cloud_corner_sharp->points[i].y,
                                   cloud_corner_sharp->points[i].z);
        Eigen::Vector3d last_point_a(
            cloud_corner_last->points[closestPointInd].x,
            cloud_corner_last->points[closestPointInd].y,
            cloud_corner_last->points[closestPointInd].z);
        Eigen::Vector3d last_point_b(cloud_corner_last->points[minPointInd2].x,
                                     cloud_corner_last->points[minPointInd2].y,
                                     cloud_corner_last->points[minPointInd2].z);

        double s;
        if (DISTORTION)
          s = (cloud_corner_sharp->points[i].intensity -
               int(cloud_corner_sharp->points[i].intensity)) /
              kScanPeriod;
        else
          s = 1.0;
        ceres::CostFunction *cost_function =
            LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
        problem.AddResidualBlock(cost_function, loss_function,
                                 r_curr2last.coeffs().data(),
                                 t_curr2last.data());
        corner_correspondence++;
      }
    }

    // find correspondence for plane features
    for (size_t i = 0; i < cloud_surf_flat->size(); ++i) {
      TransformToStart(cloud_surf_flat->points[i], pointSel,
                       *pose_estimate_curr2last);
      kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd,
                                     pointSearchSqDis);

      int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
      if (pointSearchSqDis[0] < kDistanceSqThreshold) {
        closestPointInd = pointSearchInd[0];

        // get closest point's scan ID
        int closestPointScanID =
            int(cloud_surf_last->points[closestPointInd].intensity);
        double minPointSqDis2 = kDistanceSqThreshold,
               minPointSqDis3 = kDistanceSqThreshold;

        // search in the direction of increasing scan line
        for (size_t j = closestPointInd + 1; j < cloud_surf_last->size(); ++j) {
          // if not in nearby scans, end the loop
          if (int(cloud_surf_last->points[j].intensity) >
              (closestPointScanID + kNearByScan))
            break;

          double pointSqDis = (cloud_surf_last->points[j].x - pointSel.x) *
                                  (cloud_surf_last->points[j].x - pointSel.x) +
                              (cloud_surf_last->points[j].y - pointSel.y) *
                                  (cloud_surf_last->points[j].y - pointSel.y) +
                              (cloud_surf_last->points[j].z - pointSel.z) *
                                  (cloud_surf_last->points[j].z - pointSel.z);

          // if in the same or lower scan line
          if (int(cloud_surf_last->points[j].intensity) <= closestPointScanID &&
              pointSqDis < minPointSqDis2) {
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
          }
          // if in the higher scan line
          else if (int(cloud_surf_last->points[j].intensity) >
                       closestPointScanID &&
                   pointSqDis < minPointSqDis3) {
            minPointSqDis3 = pointSqDis;
            minPointInd3 = j;
          }
        }

        // search in the direction of decreasing scan line
        for (int j = closestPointInd - 1; j >= 0; --j) {
          // if not in nearby scans, end the loop
          if (int(cloud_surf_last->points[j].intensity) <
              (closestPointScanID - kNearByScan))
            break;

          double pointSqDis = (cloud_surf_last->points[j].x - pointSel.x) *
                                  (cloud_surf_last->points[j].x - pointSel.x) +
                              (cloud_surf_last->points[j].y - pointSel.y) *
                                  (cloud_surf_last->points[j].y - pointSel.y) +
                              (cloud_surf_last->points[j].z - pointSel.z) *
                                  (cloud_surf_last->points[j].z - pointSel.z);

          // if in the same or higher scan line
          if (int(cloud_surf_last->points[j].intensity) >= closestPointScanID &&
              pointSqDis < minPointSqDis2) {
            minPointSqDis2 = pointSqDis;
            minPointInd2 = j;
          } else if (int(cloud_surf_last->points[j].intensity) <
                         closestPointScanID &&
                     pointSqDis < minPointSqDis3) {
            // find nearer point
            minPointSqDis3 = pointSqDis;
            minPointInd3 = j;
          }
        }

        if (minPointInd2 >= 0 && minPointInd3 >= 0) {
          Eigen::Vector3d curr_point(cloud_surf_flat->points[i].x,
                                     cloud_surf_flat->points[i].y,
                                     cloud_surf_flat->points[i].z);
          Eigen::Vector3d last_point_a(
              cloud_surf_last->points[closestPointInd].x,
              cloud_surf_last->points[closestPointInd].y,
              cloud_surf_last->points[closestPointInd].z);
          Eigen::Vector3d last_point_b(cloud_surf_last->points[minPointInd2].x,
                                       cloud_surf_last->points[minPointInd2].y,
                                       cloud_surf_last->points[minPointInd2].z);
          Eigen::Vector3d last_point_c(cloud_surf_last->points[minPointInd3].x,
                                       cloud_surf_last->points[minPointInd3].y,
                                       cloud_surf_last->points[minPointInd3].z);

          double s;
          if (DISTORTION)
            s = (cloud_surf_flat->points[i].intensity -
                 int(cloud_surf_flat->points[i].intensity)) /
                kScanPeriod;
          else
            s = 1.0;
          ceres::CostFunction *cost_function = LidarPlaneFactor::Create(
              curr_point, last_point_a, last_point_b, last_point_c, s);
          problem.AddResidualBlock(cost_function, loss_function,
                                   r_curr2last.coeffs().data(),
                                   t_curr2last.data());
          plane_correspondence++;
        }
      }
    }

    LOG_STEP_TIME("data association", t_data.toc());

    if ((corner_correspondence + plane_correspondence) < 10) {
      LOG(WARNING) << "coner_correspondance=" << corner_correspondence
                   << ", plane_correspondence=" << plane_correspondence;
      LOG(WARNING) << "less correspondence! "
                      "*************************************************\n";
    }

    TicToc t_solver;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 4;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG_STEP_TIME("solver time", t_solver.toc());
  }
  LOG_STEP_TIME("optimization twice", t_opt.toc());

  return true;
}