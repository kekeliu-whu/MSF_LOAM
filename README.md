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

## 2. Build MSF_LOAM
Clone the repository and catkin_make.

## 3. Velodyne VLP-16 Example
Download [NSH indoor outdoor](https://drive.google.com/file/d/1s05tBQOLNEDDurlg48KiUWxCp-YqYyGH/view) to YOUR_DATASET_FOLDER. 

```
    roslaunch msf_loam_velodyne msf_loam_velodyne_VLP_16.launch
    rosbag play YOUR_DATASET_FOLDER/nsh_indoor_outdoor.bag
```

## 4.Acknowledgements
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).
