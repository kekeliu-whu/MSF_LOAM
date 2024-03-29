#include <fmt/format.h>
#include <fmt/ostream.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common/common.h"
#include "common/tic_toc.h"
#include "proto/msg.pb.h"
#include "slam/imu_fusion/integration_base.h"
#include "slam/imu_fusion/scan_undistortion.h"
#include "slam/local/laser_mapping.h"
#include "slam/local/scan_matching/mapping_scan_matcher.h"
#include "slam/local/scan_matching/odometry_scan_matcher.h"
#include "slam/msg_conversion.h"

namespace {

const bool kEnableMapSave = true;
PointCloudOriginalPtr g_cloud_all(new PointCloudOriginal);

template <typename T>
inline typename pcl::PointCloud<T>::Ptr TransformPointCloud(const typename pcl::PointCloud<T>::ConstPtr &cloud_in,
                                                            const Rigid3d &pose) {
  typename pcl::PointCloud<T>::Ptr cloud_out(new pcl::PointCloud<T>);
  cloud_out->resize(cloud_in->size());
  for (size_t i = 0; i < cloud_in->size(); ++i)
    cloud_out->at(i) = TransformPoint(pose, cloud_in->at(i));
  return cloud_out;
}

}  // namespace

// todo
double ACC_N = 0.017, ACC_W = 0.007;
double GYR_N = 0.0033, GYR_W = 0.0012;
Eigen::Vector3d G;

LaserMapping::LaserMapping(bool is_offline_mode, proto::MsfLoamConfig config)
    : gps_fusion_handler_(std::make_shared<GpsFusion>()),
      scan_matcher_(std::make_unique<MappingScanMatcher>()),
      frame_idx_cur_(0),
      hybrid_grid_map_corner_(3.0),
      hybrid_grid_map_surf_(3.0),
      is_offline_mode_(is_offline_mode),
      is_firstframe_(true),
      should_exit_(false),
      velocity_(Vector3d::Zero()),
      config_(config),
      estimator_(FromProto(config.gravity_vector())) {
  // NodeHandle uses reference counting internally,
  // thus a local variable can be created here
  ros::NodeHandle nh;

  LOG(INFO) << "LaserMapping initializing ...";
  // get leaf size
  float line_res  = 0;
  float plane_res = 0;
  LOG_IF(WARNING, !nh.param<float>("mapping_line_resolution", line_res, 0.2))
      << "Use default mapping_line_resolution: 0.2";
  LOG_IF(WARNING, !nh.param<float>("mapping_plane_resolution", plane_res, 0.4))
      << "Use default mapping_plane_resolution: 0.4";
  LOG(INFO) << "[MAP]"
            << " line resolution " << line_res << " plane resolution "
            << plane_res;
  downsize_filter_corner_.setLeafSize(line_res, line_res, line_res);
  downsize_filter_surf_.setLeafSize(plane_res, plane_res, plane_res);

  // set publishers
  cloud_scan_origin_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/loam/origin/velodyne_cloud", 100);

  cloud_scan_publisher_        = nh.advertise<sensor_msgs::PointCloud2>("/loam/deskew/velodyne_cloud", 100);
  cloud_corner_publisher_      = nh.advertise<sensor_msgs::PointCloud2>("/loam/deskew/laser_cloud_sharp", 100);
  cloud_corner_less_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/loam/deskew/laser_cloud_less_sharp", 100);
  cloud_surf_publisher_        = nh.advertise<sensor_msgs::PointCloud2>("/loam/deskew/laser_cloud_flat", 100);
  cloud_surf_less_publisher_   = nh.advertise<sensor_msgs::PointCloud2>("/loam/deskew/laser_cloud_less_flat", 100);

  cloud_surround_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/loam/laser_cloud_surround", 100);

  aftmapped_odom_publisher_          = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
  aftmapped_odom_highfrec_publisher_ = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);
  aftmapped_path_publisher_          = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

  // RUN
  thread_ = std::thread([this] { this->Run(); });
}

LaserMapping::~LaserMapping() {
  should_exit_ = true;
  thread_.join();
  gps_fusion_handler_->Optimize();

  // save point cloud map
  if (kEnableMapSave) {
    // todo kk
    G          = estimator_.GetGravityVector();
    auto q_l_w = Quaterniond::FromTwoVectors(G, Vector3d::UnitZ());

    LOG(INFO) << "Gravity slope angle in degrees = " << q_l_w.angularDistance(Quaterniond::Identity()) * 180 / M_PI;

    for (auto &e : *g_cloud_all) {
      e.getVector3fMap() = q_l_w.cast<float>() * e.getVector3fMap();
    }

    LOG(INFO) << "Saving point cloud map ...";
    if (!g_cloud_all->empty()) {
      pcl::io::savePLYFileBinary("msf_loam_cloud.ply", *g_cloud_all);
      LOG(INFO) << "Saving done.";
    } else {
      LOG(WARNING) << "Saving skipped because point cloud is empty.";
    }
  }

  // save imu/odom data to proto file
  std::ofstream ofs(kTrajectoryPbPath, std::ios_base::out | std::ios_base::binary);
  pb_data_.SerializeToOstream(&ofs);

  LOG(INFO) << "LaserMapping finished.";
}

void LaserMapping::AddLaserOdometryResult(
    const LaserOdometryResultType &laser_odometry_result) {
  {
    absl::MutexLock lg(&mtx_odometry_result_queue_);
    odometry_result_queue_.push(laser_odometry_result);
  }
  // publish odom tf
  // high frequency publish
  nav_msgs::Odometry aftmapped_odom;
  aftmapped_odom.child_frame_id  = "aft_mapped";
  aftmapped_odom.header.frame_id = "camera_init";
  aftmapped_odom.header.stamp    = ToROS(laser_odometry_result.time);
  aftmapped_odom.pose            = ToROS(pose_odom2map_ * laser_odometry_result.odom_pose);
  aftmapped_odom_highfrec_publisher_.publish(aftmapped_odom);
}

void LaserMapping::Run() {
  while (ros::ok()) {
    LaserOdometryResultType odom_result;
    {
      absl::MutexLock lg(&mtx_odometry_result_queue_);
      // Try to get new messages in 100us, return false if no messages found or timeout,
      // Tips:
      // 1. Spurious awakening has been handled in AwaitWithTimeout
      // 2. Here we use Lock+AwaitWithTimeout instead of LockWhenWithTimeout
      auto f           = [this]() ABSL_EXCLUSIVE_LOCKS_REQUIRED(mtx_odometry_result_queue_) { return !this->odometry_result_queue_.empty(); };
      bool has_new_msg = mtx_odometry_result_queue_.AwaitWithTimeout(absl::Condition{&f}, absl::Microseconds(100));
      if (!has_new_msg) {
        if (should_exit_) {
          break;
        } else {
          continue;
        }
      }

      odom_result = odometry_result_queue_.front();
      odometry_result_queue_.pop();
      if (!is_offline_mode_) {
        while (!odometry_result_queue_.empty()) {
          LOG(WARNING)
              << "[MAP] drop lidar frame in mapping for real time performance";
          odometry_result_queue_.pop();
        }
      }
    }

    //
    // 1. deskew scan
    //
    {
      absl::MutexLock lg(&mtx_imu_buf_);
      // 如果imu没有完成初始化，就使用旋转量进行去畸变处理
      if (!estimator_.IsInitialized()) {
        UndistortScan(odom_result, imu_buf_, odom_result);
      }
    }

    pose_odom_scan2world_ = odom_result.odom_pose;

    TicToc t_whole;

    //
    // 2. match scan with surrounded map
    //
    TransformAssociateToMap();
    FilterLessFlatLessCornerFeature(odom_result, odom_result);
    MatchScan2Map(odom_result);
    TransformUpdate();

    std::shared_ptr<IntegrationBase> preintegration;
    {
      absl::MutexLock lg(&mtx_imu_buf_);
      preintegration = BuildPreintegration(imu_buf_, odom_result.time);
    }
    // 如果imu初始化完成了，就用imu预积分量计算精确的畸变值，进行去畸变处理
    if (estimator_.IsInitialized()) {
      auto DoUndistort = [&](PointCloudOriginalPtr &cloud) {
        for (auto &e : *cloud) {
          auto dt            = e.intensity;
          auto delta_qp      = GetDeltaQP(preintegration, dt);
          e.getVector3fMap() = (delta_qp.rotation() * e.getVector3fMap().cast<double>() + this->pose_odom_scan2world_.rotation().conjugate() * (velocity_ * dt - 0.5 * estimator_.GetGravityVector() * dt * dt) + delta_qp.translation())
                                   .cast<float>();
        }
      };
      DoUndistort(odom_result.cloud_full_res);
      DoUndistort(odom_result.cloud_corner_sharp);
      DoUndistort(odom_result.cloud_corner_less_sharp);
      DoUndistort(odom_result.cloud_surf_flat);
      DoUndistort(odom_result.cloud_surf_less_flat);
    }

    // 保存去畸变后的点云
    if (frame_idx_cur_ >= Estimator::kInitByFirstScanNums) {
      auto cloud = TransformPointCloud<PointTypeOriginal>(odom_result.cloud_full_res, pose_map_scan2world_);
      *g_cloud_all += *cloud;
    }

    TicToc t_add;

    //
    // 3. insert scan into map
    //
    InsertScan2Map(odom_result);

    LOG_STEP_TIME("MAP", "add points", t_add.toc());
    LOG_STEP_TIME("MAP", "whole mapping", t_whole.toc());

    gps_fusion_handler_->AddLocalPose(odom_result.time,
                                      pose_map_scan2world_);

    odom_result.map_pose = pose_map_scan2world_;
    PublishTrajectory(odom_result);

    PublishScan(odom_result);

    // todo init bias and gravity vector by set device still
    {
      absl::MutexLock lg(&mtx_imu_buf_);
      estimator_.AddData(odom_result, velocity_, imu_buf_);
      static bool is_first_init = true;
      if (estimator_.IsInitialized() && is_first_init) {
        is_first_init = false;
        G             = estimator_.GetGravityVector();
        // todo do not use cloud in init procedure
        // this->hybrid_grid_map_surf_   = HybridGrid(3.0);
        // this->hybrid_grid_map_corner_ = HybridGrid(3.0);
      }
    }

    auto odom_msg = pb_data_.add_odom_datas();
    odom_msg->set_timestamp(ToUniversal(odom_result.time));
    *odom_msg->mutable_pose()->mutable_rotation()    = ToProto(pose_map_scan2world_.rotation());
    *odom_msg->mutable_pose()->mutable_translation() = ToProto(pose_map_scan2world_.translation());

    frame_idx_cur_++;
  }
}

void LaserMapping::MatchScan2Map(const LaserOdometryResultType &odom_result) {
  PointCloudConstPtr laserCloudCornerLast = ToPointType(odom_result.cloud_corner_less_sharp);
  PointCloudConstPtr laserCloudSurfLast   = ToPointType(odom_result.cloud_surf_less_flat);

  PointCloudPtr laserCloudCornerLastStack(new PointCloud);
  downsize_filter_corner_.setInputCloud(laserCloudCornerLast);
  downsize_filter_corner_.filter(*laserCloudCornerLastStack);

  PointCloudPtr laserCloudSurfLastStack(new PointCloud);
  downsize_filter_surf_.setInputCloud(laserCloudSurfLast);
  downsize_filter_surf_.filter(*laserCloudSurfLastStack);

  TicToc t_shift;
  PointCloudPtr laserCloudCornerFromMap =
      hybrid_grid_map_corner_.GetSurroundedCloud(laserCloudCornerLast,
                                                 pose_map_scan2world_);
  PointCloudPtr laserCloudSurfFromMap =
      hybrid_grid_map_surf_.GetSurroundedCloud(laserCloudSurfLast,
                                               pose_map_scan2world_);
  LOG_STEP_TIME("MAP", "Collect surround cloud", t_shift.toc());

  LOG(INFO) << "[MAP]"
            << " corner=" << laserCloudCornerFromMap->size()
            << ", surf=" << laserCloudSurfFromMap->size();
  if (laserCloudCornerFromMap->size() > 10 &&
      laserCloudSurfFromMap->size() > 50) {
    TimestampedPointCloud<PointType> cloud_map, scan_curr;
    cloud_map.cloud_corner_less_sharp = laserCloudCornerFromMap;
    cloud_map.cloud_surf_less_flat    = laserCloudSurfFromMap;
    scan_curr.time                    = odom_result.time;
    scan_curr.cloud_corner_less_sharp = laserCloudCornerLastStack;
    scan_curr.cloud_surf_less_flat    = laserCloudSurfLastStack;
    std::shared_ptr<IntegrationBase> preintegration;
    {
      absl::MutexLock lg(&mtx_imu_buf_);
      preintegration = BuildPreintegration(imu_buf_, odom_result.time);
    }
    auto prev_state = estimator_.GetPrevState();
    {
      // todo kk
      absl::MutexLock lg{&mtx_imu_buf_};
      prev_state.imu_preintegration = BuildPreintegration(imu_buf_, prev_state.time, odom_result.time);
      velocity_                     = prev_state.v;
    }
    scan_matcher_->MatchScan2Map(cloud_map,
                                 scan_curr,
                                 estimator_.IsInitialized(),
                                 preintegration,
                                 estimator_.GetGravityVector(),
                                 prev_state,
                                 &pose_map_scan2world_,
                                 &velocity_);
  } else {
    LOG(WARNING) << "[MAP] time Map corner and surf num are not enough";
  }

  // publish surround map for every 5 frame
  if (frame_idx_cur_ % 5 == 0) {
    PointCloudPtr laserCloudSurround(new PointCloud);
    *laserCloudSurround += *laserCloudCornerFromMap;
    *laserCloudSurround += *laserCloudSurfFromMap;

    sensor_msgs::PointCloud2 laserCloudSurround3;
    pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
    laserCloudSurround3.header.stamp    = ToROS(odom_result.time);
    laserCloudSurround3.header.frame_id = "camera_init";
    cloud_surround_publisher_.publish(laserCloudSurround3);
  }
}

void LaserMapping::InsertScan2Map(const LaserOdometryResultType &odom_result) {
  hybrid_grid_map_corner_.InsertScan(
      TransformPointCloud<PointType>(ToPointType(odom_result.cloud_corner_less_sharp), pose_map_scan2world_),
      downsize_filter_corner_);

  hybrid_grid_map_surf_.InsertScan(
      TransformPointCloud<PointType>(ToPointType(odom_result.cloud_surf_less_flat), pose_map_scan2world_),
      downsize_filter_surf_);
}

void LaserMapping::FilterLessFlatLessCornerFeature(
    const LaserOdometryResultType &odom_result,
    LaserOdometryResultType &odom_result_filtered) {
  // copy all fields
  auto odom_result_filtered_out           = odom_result.CopyAllFieldsWithoudCloud();
  odom_result_filtered_out.cloud_full_res = odom_result.cloud_full_res;

  PointCloudConstPtr laserCloudCornerLast = ToPointType(odom_result.cloud_corner_less_sharp);
  PointCloudConstPtr laserCloudSurfLast   = ToPointType(odom_result.cloud_surf_less_flat);

  PointCloudPtr laserCloudCornerLastStack(new PointCloud);
  downsize_filter_corner_.setInputCloud(laserCloudCornerLast);
  downsize_filter_corner_.filter(*laserCloudCornerLastStack);
  auto indices = downsize_filter_corner_.getIndices();
  pcl::copyPointCloud(*odom_result.cloud_corner_less_sharp, *indices, *odom_result_filtered_out.cloud_corner_less_sharp);

  PointCloudPtr laserCloudSurfLastStack(new PointCloud);
  downsize_filter_surf_.setInputCloud(laserCloudSurfLast);
  downsize_filter_surf_.filter(*laserCloudSurfLastStack);
  indices = downsize_filter_corner_.getIndices();
  pcl::copyPointCloud(*odom_result.cloud_surf_less_flat, *indices, *odom_result_filtered_out.cloud_surf_less_flat);

  // handle situation when odom_result.get() == odom_result_filtered.get()
  odom_result_filtered = odom_result_filtered_out;
}

void LaserMapping::PublishTrajectory(const LaserOdometryResultType &scan) {
  nav_msgs::Odometry aftmapped_odom;
  aftmapped_odom.header.frame_id = "camera_init";
  aftmapped_odom.header.stamp    = ToROS(scan.time);
  aftmapped_odom.child_frame_id  = "aft_mapped";
  aftmapped_odom.pose            = ToROS(pose_map_scan2world_);
  aftmapped_odom_publisher_.publish(aftmapped_odom);

  geometry_msgs::PoseStamped laserAfterMappedPose;
  laserAfterMappedPose.header     = aftmapped_odom.header;
  laserAfterMappedPose.pose       = aftmapped_odom.pose.pose;
  aftmapped_path_.header.stamp    = aftmapped_odom.header.stamp;
  aftmapped_path_.header.frame_id = "camera_init";
  aftmapped_path_.poses.push_back(laserAfterMappedPose);
  aftmapped_path_publisher_.publish(aftmapped_path_);

  tf::Transform transform;
  transform.setOrigin({pose_map_scan2world_.translation().x(),
                       pose_map_scan2world_.translation().y(),
                       pose_map_scan2world_.translation().z()});
  transform.setRotation({pose_map_scan2world_.rotation().x(),
                         pose_map_scan2world_.rotation().y(),
                         pose_map_scan2world_.rotation().z(),
                         pose_map_scan2world_.rotation().w()});
  transform_broadcaster_.sendTransform(tf::StampedTransform(
      transform, aftmapped_odom.header.stamp, "/camera_init", "/aft_mapped"));
}

// todo add extrinsic parameter between imu and lidar
void LaserMapping::UndistortScan(
    const LaserOdometryResultType &laser_odometry_result,
    const std::vector<ImuData> &imu_buf,
    LaserOdometryResultType &laser_odometry_result_deskewed) {
  auto imu_preintegration = BuildPreintegration(imu_buf, laser_odometry_result.time);
  ScanUndistortionUtils::DoUndistort(laser_odometry_result, imu_preintegration, laser_odometry_result_deskewed);
}

void LaserMapping::AddImu(const ImuData &imu_data) {
  {
    absl::MutexLock lg(&mtx_imu_buf_);
    if (!imu_buf_.empty()) {
      LOG_IF(ERROR, imu_buf_.back().time >= imu_data.time);
    }
    imu_buf_.push_back(imu_data);
  }

  auto imu_msg = pb_data_.add_imu_datas();
  imu_msg->set_timestamp(ToUniversal(imu_data.time));
  *imu_msg->mutable_angular_velocity()    = ToProto(imu_data.angular_velocity);
  *imu_msg->mutable_linear_acceleration() = ToProto(imu_data.linear_acceleration);
}

void LaserMapping::PublishScan(const LaserOdometryResultType &scan) {
  sensor_msgs::PointCloud2 laser_cloud_out_msg;
  pcl::toROSMsg(*scan.cloud_full_res, laser_cloud_out_msg);
  laser_cloud_out_msg.header.stamp    = ToROS(scan.time);
  laser_cloud_out_msg.header.frame_id = "aft_mapped";
  cloud_scan_publisher_.publish(laser_cloud_out_msg);

  sensor_msgs::PointCloud2 cloud_corner_sharp_msg;
  pcl::toROSMsg(*scan.cloud_corner_sharp, cloud_corner_sharp_msg);
  cloud_corner_sharp_msg.header.stamp    = ToROS(scan.time);
  cloud_corner_sharp_msg.header.frame_id = "aft_mapped";
  cloud_corner_publisher_.publish(cloud_corner_sharp_msg);

  sensor_msgs::PointCloud2 cloud_corner_less_sharp_msg;
  pcl::toROSMsg(*scan.cloud_corner_less_sharp, cloud_corner_less_sharp_msg);
  cloud_corner_less_sharp_msg.header.stamp    = ToROS(scan.time);
  cloud_corner_less_sharp_msg.header.frame_id = "aft_mapped";
  cloud_corner_less_publisher_.publish(cloud_corner_less_sharp_msg);

  sensor_msgs::PointCloud2 cloud_surf_flat_msg;
  pcl::toROSMsg(*scan.cloud_surf_flat, cloud_surf_flat_msg);
  cloud_surf_flat_msg.header.stamp    = ToROS(scan.time);
  cloud_surf_flat_msg.header.frame_id = "aft_mapped";
  cloud_surf_publisher_.publish(cloud_surf_flat_msg);

  sensor_msgs::PointCloud2 cloud_surf_less_flat_msg;
  pcl::toROSMsg(*scan.cloud_surf_less_flat, cloud_surf_less_flat_msg);
  cloud_surf_less_flat_msg.header.stamp    = ToROS(scan.time);
  cloud_surf_less_flat_msg.header.frame_id = "aft_mapped";
  cloud_surf_less_publisher_.publish(cloud_surf_less_flat_msg);
}

void LaserMapping::AddOdom(const OdometryData &odom_data) {
#ifdef _SIM_GPS
  static int counter = 1;
  /**
   * Simulate GPS data for GPS fusion
   */
  if (counter % 10 == 0) {
    static std::default_random_engine g;
    static std::uniform_real_distribution<double> dist(-0.05, 0.05);
    gps_fusion_handler_->AddFixedPoint(
        odom_data.timestamp, Vector<double>(dist(g), dist(g), dist(g)) +
                                 odom_data.odom.translation());
  }
  ++counter;
#endif
}
