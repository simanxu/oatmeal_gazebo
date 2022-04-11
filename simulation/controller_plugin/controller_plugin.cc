#include "controller_plugin.h"

namespace {
constexpr int kNumJoints = 4;
std::vector<std::string> JointNames = {"Joint_FR", "Joint_FL", "Joint_HR", "Joint_HL"};
}  // namespace

namespace gazebo {

WorldControllerPlugin::WorldControllerPlugin() {
  q_des_ = new float[kNumJoints];
  qd_des_ = new float[kNumJoints];
  tau_des_ = new float[kNumJoints];
  for (int i = 0; i < kNumJoints; ++i) {
    q_des_[i] = 0.0;
    qd_des_[i] = 0.0;
    tau_des_[i] = 0.0;
  }
  std::cout << "Hello World!" << std::endl;
}

void WorldControllerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
  world_ = _world;
  iterations_ = 0;
  std::cout << "[CarWheel] plugin name: " << _sdf->Get<std::string>("filename") << std::endl;
  std::cout << "[CarWheel] world name: " << _world->Name() << std::endl;
}
}  // namespace gazebo
