// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <boost/progress.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "common/rigid_transform.h"

std::vector<float> read_lidar_data(const std::string lidar_data_path) {
  std::ifstream lidar_data_file(lidar_data_path,
                                std::ifstream::in | std::ifstream::binary);
  lidar_data_file.seekg(0, std::ios::end);
  const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
  lidar_data_file.seekg(0, std::ios::beg);

  std::vector<float> lidar_data_buffer(num_elements);
  lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]),
                       num_elements * sizeof(float));
  return lidar_data_buffer;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "kitti_helper");
  ros::NodeHandle n("~");
  std::string dataset_folder, sequence_number, output_bag_file;
  n.getParam("dataset_folder", dataset_folder);
  n.getParam("sequence_number", sequence_number);
  std::cout << "Reading sequence " << sequence_number << " from "
            << dataset_folder << std::endl;
  bool to_bag;
  n.getParam("to_bag", to_bag);
  if (to_bag)
    output_bag_file = dataset_folder + "sequences/" + sequence_number + ".bag";
  int publish_delay;
  n.getParam("publish_delay", publish_delay);
  publish_delay = publish_delay <= 0 ? 1 : publish_delay;

  nav_msgs::Odometry odomGT;
  odomGT.header.frame_id = "camera_init";
  odomGT.child_frame_id = "ground_truth";

  nav_msgs::Path pathGT;
  pathGT.header.frame_id = "camera_init";

  std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
  std::ifstream timestamp_file(dataset_folder + timestamp_path,
                               std::ifstream::in);

  std::string ground_truth_path = "poses/" + sequence_number + ".txt";
  std::ifstream ground_truth_file(dataset_folder + ground_truth_path,
                                  std::ifstream::in);

  std::string calib_path = "sequences/" + sequence_number + "/calib.txt";
  std::ifstream calib_file(dataset_folder + calib_path, std::ifstream::in);

  rosbag::Bag bag_out;
  if (to_bag) bag_out.open(output_bag_file, rosbag::bagmode::Write);

  // Get transform from velodyne lidar to camera
  Rigid3d Tr;
  {
    std::string line;
    // Read 5th line
    for (int i = 0; i < 5; ++i) std::getline(calib_file, line);
    if (line.find("Tr: ") == 0) {
      line.replace(0, 4, "");
    } else {
      std::cerr << "Tr parse failed!";
      exit(-1);
    }
    std::stringstream pose_stream(line);
    Eigen::Matrix<double, 3, 4> gt_pose;
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 4; ++j) {
        std::string s;
        std::getline(pose_stream, s, ' ');
        gt_pose(i, j) = stof(s);
      }
    }
    Tr.rotation() = gt_pose.topLeftCorner<3, 3>();
    Tr.translation() = gt_pose.topRightCorner<3, 1>();
  }

  std::string line;
  std::size_t line_num = 0;

  while (std::getline(timestamp_file, line) && ros::ok()) {
    float timestamp = stof(line);

    Rigid3d Tc;
    {
      std::getline(ground_truth_file, line);
      std::stringstream pose_stream(line);
      Eigen::Matrix<double, 3, 4> gt_pose;
      for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
          std::string s;
          std::getline(pose_stream, s, ' ');
          gt_pose(i, j) = stof(s);
        }
      }
      Tc.rotation() = gt_pose.topLeftCorner<3, 3>();
      Tc.translation() = gt_pose.topRightCorner<3, 1>();
    }

    Rigid3d Tl = Tr.inverse() * Tc * Tr;
    Tl.rotation().normalize();

    odomGT.header.stamp = ros::Time().fromSec(timestamp);
    odomGT.pose.pose.orientation.x = Tl.rotation().x();
    odomGT.pose.pose.orientation.y = Tl.rotation().y();
    odomGT.pose.pose.orientation.z = Tl.rotation().z();
    odomGT.pose.pose.orientation.w = Tl.rotation().w();
    odomGT.pose.pose.position.x = Tl.translation().x();
    odomGT.pose.pose.position.y = Tl.translation().y();
    odomGT.pose.pose.position.z = Tl.translation().z();

    geometry_msgs::PoseStamped poseGT;
    poseGT.header = odomGT.header;
    poseGT.pose = odomGT.pose.pose;
    pathGT.header.stamp = odomGT.header.stamp;
    pathGT.poses.push_back(poseGT);

    // read lidar point cloud
    std::stringstream lidar_data_path;
    lidar_data_path << dataset_folder
                    << "sequences/" + sequence_number + "/velodyne/"
                    << std::setfill('0') << std::setw(6) << line_num << ".bin";
    std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
    std::cout << "Frame " << line_num << ": " << lidar_data.size() / 4
              << " points." << std::endl;

    pcl::PointCloud<pcl::PointXYZI> laser_cloud;
    for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
      pcl::PointXYZI point;
      point.x = lidar_data[i];
      point.y = lidar_data[i + 1];
      point.z = lidar_data[i + 2];
      point.intensity = lidar_data[i + 3];
      laser_cloud.push_back(point);
    }

    sensor_msgs::PointCloud2 laser_cloud_msg;
    pcl::toROSMsg(laser_cloud, laser_cloud_msg);
    laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
    laser_cloud_msg.header.frame_id = "camera_init";

    if (to_bag) {
      bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
      bag_out.write("/path_gt", ros::Time::now(), pathGT);
      bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
    }

    line_num++;
  }
  bag_out.close();
  std::cout << "Done." << std::endl;

  return 0;
}