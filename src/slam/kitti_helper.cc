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
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

std::vector<float> read_lidar_data(const std::string lidar_data_path) {
  std::ifstream lidar_data_file(lidar_data_path,
                                std::ifstream::in | std::ifstream::binary);
  std::cout << lidar_data_path << std::endl;
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
  odomGT.header.frame_id = "/camera_init";
  odomGT.child_frame_id = "/ground_truth";

  nav_msgs::Path pathGT;
  pathGT.header.frame_id = "/camera_init";

  std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
  std::ifstream timestamp_file(dataset_folder + timestamp_path,
                               std::ifstream::in);

  std::string ground_truth_path = "poses/" + sequence_number + ".txt";
  std::ifstream ground_truth_file(dataset_folder + ground_truth_path,
                                  std::ifstream::in);

  rosbag::Bag bag_out;
  if (to_bag) bag_out.open(output_bag_file, rosbag::bagmode::Write);

  Eigen::Matrix3d R_transform;
  R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Eigen::Quaterniond q_transform(R_transform);

  std::string line;
  std::size_t line_num = 0;

  while (std::getline(timestamp_file, line) && ros::ok()) {
    float timestamp = stof(line);

    std::getline(ground_truth_file, line);
    std::stringstream pose_stream(line);
    std::string s;
    Eigen::Matrix<double, 3, 4> gt_pose;
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 4; ++j) {
        std::getline(pose_stream, s, ' ');
        gt_pose(i, j) = stof(s);
      }
    }

    Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
    Eigen::Quaterniond q = q_transform * q_w_i;
    q.normalize();
    Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();

    odomGT.header.stamp = ros::Time().fromSec(timestamp);
    odomGT.pose.pose.orientation.x = q.x();
    odomGT.pose.pose.orientation.y = q.y();
    odomGT.pose.pose.orientation.z = q.z();
    odomGT.pose.pose.orientation.w = q.w();
    odomGT.pose.pose.position.x = t(0);
    odomGT.pose.pose.position.y = t(1);
    odomGT.pose.pose.position.z = t(2);

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
    std::cout << "totally " << lidar_data.size() / 4
              << " points in this lidar frame." << std::endl;

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
    laser_cloud_msg.header.frame_id = "/camera_init";

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