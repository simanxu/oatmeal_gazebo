#ifndef OATMEAL_GAZEBO_COMMON_ESTIMATE_RESULT_H_
#define OATMEAL_GAZEBO_COMMON_ESTIMATE_RESULT_H_

#include "common/status_data.h"
#include "third_party/eigen/Eigen/Core"
#include "third_party/eigen/Eigen/Geometry"

namespace oatmeal {

struct ImuSensorData {
  double timestamp;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
  Eigen::Vector3d rpy;
  Eigen::Quaterniond quat;
};

struct EstimateResult {
  StatusData status;
  ImuSensorData imu_data;

  // Position and pose
  Eigen::Vector3d base_pos;

  Eigen::Vector3d base_rpy;
  Eigen::Quaterniond base_quat;
  Eigen::Matrix3d base_rot_mat;

  // Linear and angular velocity
  Eigen::Vector3d base_vel_body;
  Eigen::Vector3d base_vel_world;

  Eigen::Vector3d base_omega_body;
  Eigen::Vector3d base_omega_world;

  // Linear acceleration
  Eigen::Vector3d base_acc_body;
  Eigen::Vector3d base_acc_world;
};
}  // namespace oatmeal

#endif  // OATMEAL_GAZEBO_COMMON_ESTIMATE_RESULT_H_
