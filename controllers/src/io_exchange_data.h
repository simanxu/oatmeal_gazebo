#ifndef SRC_OATMEAL_GAZEBO_CONTROLLERS_SRC_IO_EXCHANGE_DATA_H_
#define SRC_OATMEAL_GAZEBO_CONTROLLERS_SRC_IO_EXCHANGE_DATA_H_

#include "controllers/src/oatmeal_controller.h"

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

#endif  // SRC_OATMEAL_GAZEBO_CONTROLLERS_SRC_IO_EXCHANGE_DATA_H_
