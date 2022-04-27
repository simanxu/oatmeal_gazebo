#ifndef OATMEAL_GAZEBO_COMMON_STATUS_DATA_H_
#define OATMEAL_GAZEBO_COMMON_STATUS_DATA_H_

#include "common/oatmeal_constants.h"
#include "third_party/eigen/Eigen/Core"

namespace oatmeal {

struct StatusData {
  // General cooridinate states
  Eigen::Vector<double, kNumJoints> q = Eigen::Vector<double, kNumJoints>::Zero();
  Eigen::Vector<double, kNumJoints> qd = Eigen::Vector<double, kNumJoints>::Zero();
  Eigen::Vector<double, kNumJoints> tau = Eigen::Vector<double, kNumJoints>::Zero();

  // Wheels contact data collected from sensor
  Eigen::Matrix<double, kThreeDims, kNumWheels> contact_force = Eigen::Matrix<double, kThreeDims, kNumWheels>::Zero();
};
}  // namespace oatmeal

#endif  // OATMEAL_GAZEBO_COMMON_STATUS_DATA_H_
