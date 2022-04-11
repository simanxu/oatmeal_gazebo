#include "controller_plugin.h"

namespace {
constexpr int kNumJoints = 4;
const std::string kBaseLinkNameInUrdf = "body";
std::vector<std::string> kJointNames = {"Joint_FR", "Joint_FL", "Joint_HR", "Joint_HL"};
}  // namespace

namespace gazebo {
WorldControllerPlugin::WorldControllerPlugin() {
  q_des_ = new double[kNumJoints];
  qd_des_ = new double[kNumJoints];
  tau_des_ = new double[kNumJoints];
  joint_list_ = new physics::JointPtr[kNumJoints];
  for (int i = 0; i < kNumJoints; ++i) {
    q_des_[i] = 0.0;
    qd_des_[i] = 0.0;
    tau_des_[i] = 0.0;
    joint_list_[i] = NULL;
  }
  std::cout << "Hello World!" << std::endl;
}

void WorldControllerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
  this->world_ = _world;
  this->model_ = NULL;
  this->iterations_ = 0;

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateEnd(std::bind(&WorldControllerPlugin::OnUpdateEnd, this));

  std::cout << "[CarWheel] plugin name: " << _sdf->Get<std::string>("filename") << std::endl;
  std::cout << "[CarWheel] world name: " << _world->Name() << std::endl;
}

void WorldControllerPlugin::OnUpdateEnd() {
  // Check the gazebo model status
  bool init_status = true;
  if (!model_) {
    init_status = InitModel();
    std::cout << "re-init model " << init_status << std::endl;
  }
  if (!init_status) {
    model_ = NULL;
    return;
  }

  // Update the sensor status from gazebo
  // this->updateSensorsStatus();

  // Exchange control and sensor data between controller and gazebo
  // ioExchangeData result = this->updateFSMcontroller();

  for (int i = 0; i < kNumJoints; ++i) {
    // ref_jnt_q[i] = result.sim_controller_pos[k];
    q_des_[i] = this->iterations_ * 0.01;
    tau_des_[i] = 3.0;
  }

  // this->updatePositionController();

  this->updateTorqueController();

  this->iterations_++;
}

// [in]	_index	Index of the joint axis (degree of freedom).
// 对于一般的关节，其自由度都是1，因此_index参数一般设置为0
// [in]	_position	Position to set the joint to. unspecified, pure kinematic teleportation.
void WorldControllerPlugin::updatePositionController() {
  for (int i = 0; i < kNumJoints; ++i) {
    joint_list_[i]->SetPosition(0, q_des_[i], false);
  }

  // for (int i = 0; i < kNumJoints; ++i) {
  //   // Velocity API based
  //   double vel_limit = joint_ptr_list[i]->GetVelocityLimit(0);
  //   double pos_lower_limit = joint_ptr_list[i]->LowerLimit(0);
  //   double pos_upper_limit = joint_ptr_list[i]->UpperLimit(0);
  //   double effort_limit = joint_ptr_list[i]->GetEffortLimit(0);
  //   joint_ptr_list[i]->SetParam("fmax", 0, (double)effort_limit);
  //   ref_jnt_q[i] = ref_jnt_q[i] > pos_upper_limit ? pos_upper_limit : ref_jnt_q[i];
  //   ref_jnt_q[i] = ref_jnt_q[i] < pos_lower_limit ? pos_lower_limit : ref_jnt_q[i];
  //   double error_curr = ref_jnt_q[i] - act_jnt_q[i];
  //   // Kp is set to 1000 as default?
  //   double vel_input = 1000.0 * error_curr;
  //   vel_input = vel_input > vel_limit ? vel_limit : vel_input;
  //   vel_input = vel_input < -vel_limit ? -vel_limit : vel_input;

  //   joint_ptr_list[i]->SetParam("vel", 0, (double)(vel_input));
  // }
}

void WorldControllerPlugin::updateTorqueController() {
  for (int i = 0; i < kNumJoints; ++i) {
    // tau_[i] = result.sim_controller_tau[i];
    joint_list_[i]->SetForce(0, tau_des_[i]);
  }
}

void WorldControllerPlugin::Reset() { InitModel(); }

bool WorldControllerPlugin::InitModel() {
  std::string urdf_file_name;
  int model_counts = world_->ModelCount();
  std::cout << "Found " << model_counts << " models" << std::endl;
  for (int i = 0; i < model_counts; ++i) {
    std::cout << i << " " << world_->ModelByIndex(i)->GetName() << std::endl;
    if (world_->ModelByIndex(i)->GetLink(kBaseLinkNameInUrdf) != NULL) {
      model_ = world_->ModelByIndex(i);
      urdf_file_name = world_->ModelByIndex(i)->GetName();
      std::cout << "urdf file name: " << urdf_file_name << std::endl;
      break;
    }
  }

  if (model_ != NULL) {
    for (int i = 0; i < kNumJoints; ++i) {
      joint_list_[i] = model_->GetJoint(kJointNames[i]);
    }
    std::cout << "Success init model!" << std::endl;
    return true;
  } else {
    std::cout << "We cannot found proper model, init plugin failed!" << std::endl;
    return false;
  }
}

}  // namespace gazebo
