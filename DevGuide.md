# MSF_LOAM
## Multi-Sensor Fusion SLAM

MSF_LOAM is a multi-sensor fusion SLAM implementation based on [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).

<img src="https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/picture/kitti.png" width = 55% height = 55%/>

**Modifier** [Keke Liu](kekliu@outlook.com)

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Recommend: Ubuntu 18.04 and [ROS melodic](http://wiki.ros.org/ROS/Installation).

### 1.2. **Ceres Solver**
```shell
sudo apt install libceres-dev
```

### 1.3. **PCL**
```shell
sudo apt install libpcl-dev
```

## 2. Build MSF_LOAM and load environment
```bash
git clone https://gitee.com/kekliu/MSF_LOAM.git
cd MSF_LOAM
mkdir build && cd build
cmake ..
make
source devel/setup.bash
```

## 3. Velodyne VLP-16 Example
Download [NSH indoor outdoor](https://drive.google.com/file/d/1s05tBQOLNEDDurlg48KiUWxCp-YqYyGH/view) to YOUR_DATASET_FOLDER. 

```
    roslaunch msf_loam_velodyne msf_loam_velodyne_VLP_16.launch
    rosbag play YOUR_DATASET_FOLDER/nsh_indoor_outdoor.bag
```

## 4. 开发者指南
### 4.1 可执行程序介绍
#### msf_loam_node
```
功能：SLAM  
命令：./msf_loam_node
参数：  
    -bag_filename (Bag file to read in offline mode.) type: string default: ""  
    -is_offline_mode (Runtime mode: online or offline.) type: bool  default: false  
输出：用户可打开rviz接收该节点发布的各种话题，rviz配置文件在rviz_cfg/中；程序的所有中间和最终输出，包含算法各阶段运行时间统计、融合IMU、融合DGPS等，都以日志的形式同时输出到标准输出和/tmp/msf_loam_node*.log文件中，请及时导出。
注意：默认处理16线雷达数据，若要处理64线数据，请提前运行`rosparam set scan_line 64`。
```
#### kittiHelper
```
功能：将kitti数据集中的Sequence转换为.bag格式的文件以供SLAM处理
命令：roslaunch msf_loam_velodyne kitti_helper.launch
配置：kitti_helper.launch为配置文件，运行格式转换前需要修改
```
### 4.2 实时模式和后处理模式
实时模式：LiDAR Mapping线程实时处理点云消息，处理不过来就丢弃，使用示例：./msf_loam_node -is_offline_mode false（同时rosbag play \<path-to-bag-filename\>）  
后处理模式：LiDAR Mapping处理所有点云消息，使用示例：./msf_loam_node -is_offline_mode true -bag_filename \<path-to-bag-filename\>
### 4.3 STGM
LaserMapping类中的成员变量hybrid_grid_map_corner_和hybrid_grid_map_surf_结构为STGM地图，初始化时的参数为STGM地图的格网大小。
### 4.4 DGPS
只要bag文件中有名为/odometry_gt的topic，则程序自动使用该真实轨迹模拟1Hz、5cm的DGPS，并在SLAM运行结束后进行DGPS融合。
### 4.5 IMU
打开laser_odometry.cc，找到`pose_curr2last_.rotation() = scan_last_.imu_rotation * scan_curr.imu_rotation.inverse();`这一行，取消注释即可融合IMU。

## 5.Acknowledgements
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).
