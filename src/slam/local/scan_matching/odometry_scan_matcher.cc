#include <ceres/ceres.h>
#include <pcl/common/copy_point.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sstream>
#include <string>

#include "common/common.h"
#include "common/tic_toc.h"
#include "lidar_factor.h"
#include "odometry_scan_matcher.h"

namespace {

constexpr double kScanPeriod          = 0.1;
constexpr double kDistanceSqThreshold = 25;
constexpr double kNearByScan          = 2.5;
constexpr int kOptimalNum             = 2;

// undistort lidar point
void TransformToStart(const PointTypeOriginal &point_in, PointTypeOriginal &point_out,
                      const Rigid3d &transform_curr2last) {
  // interpolation ratio
  double s = 1.0;
  Eigen::Quaterniond q_point_last =
      Eigen::Quaterniond::Identity().slerp(s, transform_curr2last.rotation());
  Eigen::Vector3d t_point_last = s * transform_curr2last.translation();
  Eigen::Vector3d point(point_in.x, point_in.y, point_in.z);
  Eigen::Vector3d un_point = q_point_last * point + t_point_last;

  pcl::copyPoint(point_in, point_out);
  point_out.getVector3fMap() = un_point.cast<float>();
}

PointType ToPointType(const PointTypeOriginal &point_in) {
  PointType point_out;
  pcl::copyPoint(point_in, point_out);
  return point_out;
}

}  // namespace

bool OdometryScanMatcher::MatchScan2Scan(const TimestampedPointCloud<PointTypeOriginal> &scan_last,
                                         const TimestampedPointCloud<PointTypeOriginal> &scan_curr,
                                         Rigid3d *pose_estimate_curr2last) {
  auto cloud_corner_sharp      = scan_curr.cloud_corner_sharp;
  auto cloud_corner_less_sharp = scan_curr.cloud_corner_less_sharp;
  auto cloud_surf_flat         = scan_curr.cloud_surf_flat;
  auto cloud_surf_less_flat    = scan_curr.cloud_surf_less_flat;
  auto cloud_corner_last       = scan_last.cloud_corner_less_sharp;
  auto cloud_surf_last         = scan_last.cloud_surf_less_flat;

  TicToc t_opt;

  TicToc t_kdtree;
  // less sharp 点构造的 kdtree
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_last(new pcl::KdTreeFLANN<PointType>());
  kdtree_corner_last->setInputCloud(ToPointType(cloud_corner_last));
  // less flat 点构造的 kdtree
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_last(new pcl::KdTreeFLANN<PointType>());
  kdtree_surf_last->setInputCloud(ToPointType(cloud_surf_last));
  LOG_STEP_TIME("ODO", "Build kdtree", t_kdtree.toc());

  for (size_t opti_counter = 0; opti_counter < kOptimalNum; ++opti_counter) {
    int corner_correspondence = 0, plane_correspondence = 0;

    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization =
        new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;

    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(
        pose_estimate_curr2last->rotation().coeffs().data(), 4,
        q_parameterization);
    problem.AddParameterBlock(pose_estimate_curr2last->translation().data(), 3);

    PointTypeOriginal pointSel;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    TicToc t_data;
    // find correspondence for corner features
    for (size_t i = 0; i < cloud_corner_sharp->size(); ++i) {
      TransformToStart(cloud_corner_sharp->points[i], pointSel,
                       *pose_estimate_curr2last);
      kdtree_corner_last->nearestKSearch(ToPointType(pointSel), 1, pointSearchInd, pointSearchSqDis);

      int closestPointInd = -1, minPointInd2 = -1;
      if (pointSearchSqDis[0] < kDistanceSqThreshold) {
        closestPointInd        = pointSearchInd[0];
        int closestPointScanID = cloud_corner_last->points[closestPointInd].ring;

        double minPointSqDis2 = kDistanceSqThreshold;
        // search in the direction of increasing scan line
        for (int j = closestPointInd + 1; j < (int)cloud_corner_last->size(); ++j) {
          // if in the same scan line, continue
          if (cloud_corner_last->points[j].ring <= closestPointScanID)
            continue;

          // if not in nearby scans, end the loop
          if (cloud_corner_last->points[j].ring > closestPointScanID + kNearByScan)
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
            minPointInd2   = j;
          }
        }

        // search in the direction of decreasing scan line
        for (int j = closestPointInd - 1; j >= 0; --j) {
          // if in the same scan line, continue
          if (cloud_corner_last->points[j].ring >= closestPointScanID)
            continue;

          // if not in nearby scans, end the loop
          if (cloud_corner_last->points[j].ring < closestPointScanID - kNearByScan)
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
            minPointInd2   = j;
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

        double s = 1.0;
        ceres::CostFunction *cost_function =
            LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
        problem.AddResidualBlock(
            cost_function, loss_function,
            pose_estimate_curr2last->rotation().coeffs().data(),
            pose_estimate_curr2last->translation().data());
        corner_correspondence++;
      }
    }

    // find correspondence for plane features
    for (size_t i = 0; i < cloud_surf_flat->size(); ++i) {
      TransformToStart(cloud_surf_flat->points[i], pointSel,
                       *pose_estimate_curr2last);
      kdtree_surf_last->nearestKSearch(ToPointType(pointSel), 1, pointSearchInd,
                                       pointSearchSqDis);

      int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
      if (pointSearchSqDis[0] < kDistanceSqThreshold) {
        closestPointInd = pointSearchInd[0];

        // get closest point's scan ID
        int closestPointScanID =
            cloud_surf_last->points[closestPointInd].ring;
        double minPointSqDis2 = kDistanceSqThreshold,
               minPointSqDis3 = kDistanceSqThreshold;

        // search in the direction of increasing scan line
        for (size_t j = closestPointInd + 1; j < cloud_surf_last->size(); ++j) {
          // if not in nearby scans, end the loop
          if (cloud_surf_last->points[j].ring >
              (closestPointScanID + kNearByScan))
            break;

          double pointSqDis = (cloud_surf_last->points[j].x - pointSel.x) *
                                  (cloud_surf_last->points[j].x - pointSel.x) +
                              (cloud_surf_last->points[j].y - pointSel.y) *
                                  (cloud_surf_last->points[j].y - pointSel.y) +
                              (cloud_surf_last->points[j].z - pointSel.z) *
                                  (cloud_surf_last->points[j].z - pointSel.z);

          // if in the same or lower scan line
          if (cloud_surf_last->points[j].ring <= closestPointScanID && pointSqDis < minPointSqDis2) {
            minPointSqDis2 = pointSqDis;
            minPointInd2   = j;
          }
          // if in the higher scan line
          else if (cloud_surf_last->points[j].ring > closestPointScanID &&
                   pointSqDis < minPointSqDis3) {
            minPointSqDis3 = pointSqDis;
            minPointInd3   = j;
          }
        }

        // search in the direction of decreasing scan line
        for (int j = closestPointInd - 1; j >= 0; --j) {
          // if not in nearby scans, end the loop
          if (cloud_surf_last->points[j].ring < (closestPointScanID - kNearByScan))
            break;

          double pointSqDis = (cloud_surf_last->points[j].x - pointSel.x) *
                                  (cloud_surf_last->points[j].x - pointSel.x) +
                              (cloud_surf_last->points[j].y - pointSel.y) *
                                  (cloud_surf_last->points[j].y - pointSel.y) +
                              (cloud_surf_last->points[j].z - pointSel.z) *
                                  (cloud_surf_last->points[j].z - pointSel.z);

          // if in the same or higher scan line
          if (cloud_surf_last->points[j].ring >= closestPointScanID && pointSqDis < minPointSqDis2) {
            minPointSqDis2 = pointSqDis;
            minPointInd2   = j;
          } else if (cloud_surf_last->points[j].ring < closestPointScanID &&
                     pointSqDis < minPointSqDis3) {
            // find nearer point
            minPointSqDis3 = pointSqDis;
            minPointInd3   = j;
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

          double s                           = 1.0;
          ceres::CostFunction *cost_function = LidarPlaneFactor::Create(
              curr_point, last_point_a, last_point_b, last_point_c, s);
          problem.AddResidualBlock(
              cost_function, loss_function,
              pose_estimate_curr2last->rotation().coeffs().data(),
              pose_estimate_curr2last->translation().data());
          plane_correspondence++;
        }
      }
    }

    LOG_STEP_TIME("ODO", "Data association", t_data.toc());

    if ((corner_correspondence + plane_correspondence) < 10) {
      LOG(WARNING) << "[MAP] less correspondence: corner_correspondence="
                   << corner_correspondence
                   << ", plane_correspondence=" << plane_correspondence;
      return false;
    }

    TicToc t_solver;
    ceres::Solver::Options options;
    options.max_num_iterations           = 6;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (opti_counter == kOptimalNum - 1) {
      this->RefinePoseByRejectOutliers(problem);
    }
    LOG_STEP_TIME("ODO", "Solver time", t_solver.toc());
  }
  LOG_STEP_TIME("ODO", "Optimization twice", t_opt.toc());

  return true;
}