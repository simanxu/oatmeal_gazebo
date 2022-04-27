#ifndef OATMEAL_GAZEBO_COMMON_CONTROL_COMMAND_H_
#define OATMEAL_GAZEBO_COMMON_CONTROL_COMMAND_H_

#include "common/oatmeal_constants.h"
#include "third_party/eigen/Eigen/Core"

namespace oatmeal {

enum ControlMode {
  kPassive = 0,
  kNorminalController,
  kMpcController,
  kWbcController,
};

struct JoystickCmd {
  double axes[6];
  int buttons[16];
};

struct ControlCommand {
  // 上层控制指令：机器人状态机
  int state_id;

  // 中层控制指令：来自手柄、遥控器的控制指令
  JoystickCmd joy_cmd;

  // 底层控制指令：针对机器人硬件部分的控制指令
  Eigen::Vector<double, kNumJoints> tau = Eigen::Vector<double, kNumJoints>::Zero();
  Eigen::Vector<double, kNumJoints> tau_ff = Eigen::Vector<double, kNumJoints>::Zero();

  Eigen::Vector<double, kNumJoints> kp = Eigen::Vector<double, kNumJoints>::Zero();
  Eigen::Vector<double, kNumJoints> kd = Eigen::Vector<double, kNumJoints>::Zero();
  Eigen::Vector<double, kNumJoints> q_des = Eigen::Vector<double, kNumJoints>::Zero();
  Eigen::Vector<double, kNumJoints> qd_des = Eigen::Vector<double, kNumJoints>::Zero();

  Eigen::Vector<bool, kNumJoints> enable_motor = Eigen::Vector<bool, kNumJoints>::Zero();
};

}  // namespace oatmeal

#endif  // OATMEAL_GAZEBO_COMMON_CONTROL_COMMAND_H_
