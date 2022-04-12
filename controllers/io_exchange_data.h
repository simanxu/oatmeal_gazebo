#ifndef OATMEAL_GAZEBO_CONTROLLERS_IO_EXCHANGE_DATA_H_
#define OATMEAL_GAZEBO_CONTROLLERS_IO_EXCHANGE_DATA_H_

enum ControlMode {
  kStop = 0,
  kPosition,
  kVelocity,
  kForce,
};

struct JoystickCMD {
  double axes[6];
  int buttons[9];
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

#endif  // OATMEAL_GAZEBO_CONTROLLERS_IO_EXCHANGE_DATA_H_
