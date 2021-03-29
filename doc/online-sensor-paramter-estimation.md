# Online sensor parameter estimation
[toc]

## IMU instrinsic parameters
IMU instrinsic parameters such as gyro bias can be calibrated by preintegration.

## IMU-Lidar extrinsic parameters
$$ q^{b_k}_{l_{k+1}}=q^{b_k}_{b_{k+1}}\otimes q^b_l=q^b_l\otimes q^{l_k}_{l_{k+1}} $$  

$$ \left(L(q^{b_k}_{b_{k+1}})-R(q^{l_k}_{l_{k+1}})\right)q_l^b=0 $$

## IMU-Lidar time offset
### Prerequisites
* Timestamp in IMU and Lidar is accurate, respectively
* Timeporal misalignment remains constant
* Timeporal misalignment may change due to IMU clock drift.

### Principe
TODO
