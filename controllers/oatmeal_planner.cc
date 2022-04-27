#include "controllers/oatmeal_planner.h"

namespace oatmeal {
OatmealPlanner::OatmealPlanner() {}

OatmealPlanner::~OatmealPlanner() {}

// void OatmealPlanner::UpdateCommandInput() {
//   // Update joy status
//   for (int i = 0; i < 9; i++) {
//     sim_joy_cmd_.buttons[i] = msgIn->buttons[i];
//   }
//   for (int i = 0; i < 6; i++) {
//     sim_joy_cmd_.axes[i] = msgIn->axes[i];
//   }

//   // Update control mode
//   if (sim_joy_cmd_.buttons[0] == 1) {
//     if (control_mode_ != ControlMode::kPassive) std::cout << "Switch to Passive!" << std::endl;
//     control_mode_ = ControlMode::kPassive;
//   } else if (sim_joy_cmd_.buttons[1] == 1) {
//     if (control_mode_ != ControlMode::kNorminalController) std::cout << "Switch to Norminal Controller!" <<
//     std::endl; control_mode_ = ControlMode::kNorminalController;
//   } else if (sim_joy_cmd_.buttons[2] == 1) {
//     if (control_mode_ != ControlMode::kMpcController) std::cout << "Switch to MPC Controller!" << std::endl;
//     control_mode_ = ControlMode::kMpcController;
//   } else if (sim_joy_cmd_.buttons[3] == 1) {
//     if (control_mode_ != ControlMode::kWbcController) std::cout << "Switch to WBC Controller!" << std::endl;
//     control_mode_ = ControlMode::kWbcController;
//   } else {
//     if (control_mode_ != ControlMode::kPassive) std::cout << "Switch to Passive!" << std::endl;
//     control_mode_ = ControlMode::kPassive;
//   }

//   // Update control command
//   base_linear_vel_des_.x() = 1.0 * sim_joy_cmd_.axes[1];
//   base_linear_vel_des_.y() = 1.0 * sim_joy_cmd_.axes[0];
//   base_linear_vel_des_.z() = base_linear_vel_act_.z();
// }

bool OatmealPlanner::CarVelocityControlTask() { return true; }

bool OatmealPlanner::PendulumSwingTask() { return true; }
}  // namespace oatmeal
