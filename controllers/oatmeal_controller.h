#ifndef SRC_OATMEAL_GAZEBO_CONTROLLERS_OATMEAL_CONTROLLER_H_
#define SRC_OATMEAL_GAZEBO_CONTROLLERS_OATMEAL_CONTROLLER_H_

#include "third_party/eigen/Eigen/Dense"
#include "third_party/psopt/include/psopt.h"
#include "third_party/qpOASES/include/qpOASES.hpp"
#include "third_party/rbdl/include/rbdl/rbdl.h"

enum ControlMode {
  kPassive = 0,
  kNorminalController,
  kMpcController,
  kWbcController,
};

struct JoystickCMD {
  double axes[6];
  int buttons[16];
};

struct ioExchangeData {
  //   xManDyn::Model* ptr_x_dyn;
  //   EstimateResult* estimate_result;
  //   ControlCommand* control_command;

  // Left and right foot contact detection
  //   int foot_contact_counts[4];
  //   Eigen::VectorXd real_motor_tau;
  double sim_controller_tau[4];
  double sim_controller_pos[4];
};

#endif  // SRC_OATMEAL_GAZEBO_CONTROLLERS_OATMEAL_CONTROLLER_H_
