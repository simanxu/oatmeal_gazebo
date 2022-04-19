#include "simulation/controller_plugin/controller_plugin.h"

namespace {

constexpr int kNumJoints = 4;
constexpr double kWheelRadius = 0.05;
const std::string kBaseName = "real_body";  // used in carwheel.urdf
// const std::string kBaseName = "virtual_body";  // used in carwheel_constraint.urdf
std::vector<std::string> kJointNames = {"Joint_FR", "Joint_FL", "Joint_HR", "Joint_HL"};
}  // namespace

namespace gazebo {
WorldControllerPlugin::WorldControllerPlugin() {
  q_act_ = new double[kNumJoints];
  qd_act_ = new double[kNumJoints];
  tau_act_ = new double[kNumJoints];

  q_des_ = new double[kNumJoints];
  qd_des_ = new double[kNumJoints];
  tau_des_ = new double[kNumJoints];

  joint_list_ = new physics::JointPtr[kNumJoints];
  for (int i = 0; i < kNumJoints; ++i) {
    q_act_[i] = 0.0;
    qd_act_[i] = 0.0;
    tau_act_[i] = 0.0;

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
  this->control_dt_ = 0.001;

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateEnd(std::bind(&WorldControllerPlugin::OnUpdateEnd, this));

  // Create ros thread
  node_handle_.setCallbackQueue(&callback_queue_);

  cmd_ros_thread_ = new std::thread(&WorldControllerPlugin::RosThreadForCmdMsg, this);
  // detach()的作用是将子线程和主线程的关联分离，也就是说detach()后子线程在后台独立继续运行，主线程无法再取得子线程的控制权，即使主线程结束，子线程未执行也不会结束
  cmd_ros_thread_->detach();

  pub_ros_thread_ = new std::thread(&WorldControllerPlugin::RosThreadForLogMsg, this);
  pub_ros_thread_->detach();

  control_mode_ = ControlMode::kPassive;
}

// Create a ros thread for msg and cmd send
// Avoid prohram block by ros
void WorldControllerPlugin::RosThreadForCmdMsg() {
  joy_stick_sub_ = node_handle_.subscribe("/joy", 10, &WorldControllerPlugin::UpdateJoyStatus, this);
  ros::Rate loopRate(1000);
  while (ros::ok()) {
    // spinOnce()循环等待订阅节点的回调函数
    ros::spinOnce();
    loopRate.sleep();
  }
}

void WorldControllerPlugin::RosThreadForLogMsg() {
  data_publisher_ = node_handle_.advertise<std_msgs::Int64MultiArray>("/carwheel/status", 1000);
  ros::Rate loopRate(1000);
  std_msgs::Int64MultiArray msgs;
  msgs.data.resize(2);
  while (ros::ok()) {
    msgs.data[0] = this->iterations_;
    msgs.data[1] = sim_joy_cmd_.buttons[0];
    data_publisher_.publish(msgs);
    loopRate.sleep();
  }
}

void WorldControllerPlugin::UpdateJoyStatus(const sensor_msgs::Joy::ConstPtr& msgIn) {
  // Update joy status
  for (int i = 0; i < 9; i++) {
    sim_joy_cmd_.buttons[i] = msgIn->buttons[i];
  }
  for (int i = 0; i < 6; i++) {
    sim_joy_cmd_.axes[i] = msgIn->axes[i];
  }

  // Update control mode
  if (sim_joy_cmd_.buttons[0] == 1) {
    if (control_mode_ != ControlMode::kPassive) std::cout << "Switch to Passive!" << std::endl;
    control_mode_ = ControlMode::kPassive;
  } else if (sim_joy_cmd_.buttons[1] == 1) {
    if (control_mode_ != ControlMode::kNorminalController) std::cout << "Switch to Norminal Controller!" << std::endl;
    control_mode_ = ControlMode::kNorminalController;
  } else if (sim_joy_cmd_.buttons[2] == 1) {
    if (control_mode_ != ControlMode::kMpcController) std::cout << "Switch to MPC Controller!" << std::endl;
    control_mode_ = ControlMode::kMpcController;
  } else if (sim_joy_cmd_.buttons[3] == 1) {
    if (control_mode_ != ControlMode::kWbcController) std::cout << "Switch to WBC Controller!" << std::endl;
    control_mode_ = ControlMode::kWbcController;
  } else {
    control_mode_ = ControlMode::kPassive;
  }

  // Update control command
  base_linear_vel_des_.x() = 10.0 * sim_joy_cmd_.axes[1];
  base_linear_vel_des_.y() = 10.0 * sim_joy_cmd_.axes[0];
  base_linear_vel_des_.z() = base_linear_vel_act_.z();
}

void WorldControllerPlugin::OnUpdateEnd() {
  // Check the Gazebo model status
  bool init_status = true;
  if (!model_) {
    init_status = InitModel();
  }
  if (!init_status) {
    model_ = NULL;
    return;
  }

  callback_queue_.callAvailable();

  // Update the sensor status from gazebo
  this->UpdateSensorsStatus();

  // Exchange control and sensor data between controller and gazebo
  // ioExchangeData result = this->updateFSMcontroller();
  if (control_mode_ == ControlMode::kPassive) {
    for (int i = 0; i < kNumJoints; ++i) {
      q_des_[i] = q_act_[i];
    }
  } else if (control_mode_ == ControlMode::kNorminalController) {
    // omega = v / (2 * pi * R)
    double omega = base_linear_vel_des_.x() / (2 * M_PI * kWheelRadius);
    for (int i = 0; i < kNumJoints; ++i) {
      q_des_[i] = q_act_[i] + omega * control_dt_;
    }
    fprintf(stderr, "%3.8f %3.8f %3.8f %3.8f\n", omega, q_act_[0], q_des_[0], q_des_[0] - q_act_[0]);
  } else if (control_mode_ == ControlMode::kMpcController) {
    std::cout << "Undefined controller!" << std::endl;
  } else if (control_mode_ == ControlMode::kWbcController) {
    std::cout << "Undefined controller!" << std::endl;
  }

  // Gazebo controller: 可以用位置、速度、力矩模式实现Gazebo底层的控制指令下发
  this->UpdateVelocityApiBasedController();

  this->iterations_++;
}

void WorldControllerPlugin::UpdateSensorsStatus() {
  // Update joint sensors status
  bool is_sensor_return_nan = false;
  for (int i = 0; i < kNumJoints; ++i) {
    q_act_[i] = joint_list_[i]->Position(0);
    qd_act_[i] = joint_list_[i]->GetVelocity(0);
    tau_act_[i] = joint_list_[i]->GetForce(0);

    if (std::isnan(q_act_[i])) {
      is_sensor_return_nan = true;
      q_act_[i] = 0.0;
      // std::cout << "nan in joint q " << i << std::endl;
    }

    if (std::isnan(qd_act_[i])) {
      is_sensor_return_nan = true;
      qd_act_[i] = 0.0;
      // std::cout << "nan in joint qd " << i << std::endl;
    }

    if (std::isnan(tau_act_[i])) {
      is_sensor_return_nan = true;
      tau_act_[i] = 0.0;
      // std::cout << "nan in joint tau " << i << std::endl;
    }
  }
  // std::cout << "success update joint sensors status!" << std::endl;

  // Update fake IMU status
  base_quat_act_.w() = base_link_->WorldPose().Rot().W();
  base_quat_act_.x() = base_link_->WorldPose().Rot().X();
  base_quat_act_.y() = base_link_->WorldPose().Rot().Y();
  base_quat_act_.z() = base_link_->WorldPose().Rot().Z();

  base_xyz_act_.x() = base_link_->WorldPose().Pos().X();
  base_xyz_act_.y() = base_link_->WorldPose().Pos().Y();
  base_xyz_act_.z() = base_link_->WorldPose().Pos().Z();
  // std::cout << "World position: " << base_xyz_act_.transpose() << std::endl;

  base_linear_vel_act_.x() = base_link_->WorldLinearVel().X();
  base_linear_vel_act_.y() = base_link_->WorldLinearVel().Y();
  base_linear_vel_act_.z() = base_link_->WorldLinearVel().Z();

  // Confirm whether you need absolute angular velocity(in world frame) or relative angular velocity(in local frame)
  // WorldAngularVel = RotMat * RelativeAngularVel <==> RotMat.transpose() * WorldAngularVel = RelativeAngularVel
  base_angular_vel_act_.x() = base_link_->WorldAngularVel().X();
  base_angular_vel_act_.y() = base_link_->WorldAngularVel().Y();
  base_angular_vel_act_.z() = base_link_->WorldAngularVel().Z();
  // std::cout << "World Angular Vel act: " << base_angular_vel_act_.transpose() << std::endl;
  // std::cout << "Local Angular Vel cal: "
  //           << (base_quat_act_.toRotationMatrix().transpose() * base_angular_vel_act_).transpose() << std::endl;

  base_angular_vel_act_.x() = base_link_->RelativeAngularVel().X();
  base_angular_vel_act_.y() = base_link_->RelativeAngularVel().Y();
  base_angular_vel_act_.z() = base_link_->RelativeAngularVel().Z();
  // std::cout << "Local Angular Vel act: " << base_angular_vel_act_.transpose() << std::endl;

  base_linear_acc_act_.x() = base_link_->WorldLinearAccel().X();
  base_linear_acc_act_.y() = base_link_->WorldLinearAccel().Y();
  base_linear_acc_act_.z() = base_link_->WorldLinearAccel().Z();

  // std::cout << "success update fake IMU sensor status!" << std::endl;
}

// SetPosition(index, pos, bool)
// index is the index of the joint axis (degree of freedom), 一般关节自由度都是1，因此index为0
void WorldControllerPlugin::UpdatePositionApiBasedController() {
  bool set_cmd_success = true;
  for (int i = 0; i < kNumJoints; ++i) {
    set_cmd_success = joint_list_[i]->SetPosition(0, q_des_[i], false);
    if (!set_cmd_success) {
      std::cout << "set position cmd failed at joint " << i << std::endl;
    }
  }
}

// Velocity API based controller
void WorldControllerPlugin::UpdateVelocityApiBasedController() {
  for (int i = 0; i < kNumJoints; ++i) {
    double vel_limit = joint_list_[i]->GetVelocityLimit(0);
    double pos_lower_limit = joint_list_[i]->LowerLimit(0);
    double pos_upper_limit = joint_list_[i]->UpperLimit(0);
    double effort_limit = joint_list_[i]->GetEffortLimit(0);
    joint_list_[i]->SetParam("fmax", 0, effort_limit);
    q_des_[i] = std::clamp(q_des_[i], pos_lower_limit, pos_upper_limit);
    // Kp is set to 1000 as default ?
    double vel_input = (q_des_[i] - q_act_[i]) / control_dt_;
    vel_input = std::clamp(vel_input, -vel_limit, vel_limit);
    joint_list_[i]->SetVelocity(0, vel_input);
    std::cout << i << " vel_input: " << vel_input << std::endl;
  }
}

void WorldControllerPlugin::UpdateForceApiBasedController() {
  for (int i = 0; i < kNumJoints; ++i) {
    // tau_[i] = result.sim_controller_tau[i];
    joint_list_[i]->SetForce(0, tau_des_[i]);
  }
}

bool WorldControllerPlugin::InitModel() {
  std::string urdf_file_name;
  for (int i = 0; i < world_->ModelCount(); ++i) {
    std::vector<physics::LinkPtr> link_vec = world_->ModelByIndex(i)->GetLinks();
    for (int n = 0; n < link_vec.size(); ++n) {
      std::cout << "model " << world_->ModelByIndex(i)->GetName() << ", link " << link_vec[n]->GetName() << std::endl;
    }
    if (world_->ModelByIndex(i)->GetLink(kBaseName) != NULL) {
      model_ = world_->ModelByIndex(i);
      urdf_file_name = world_->ModelByIndex(i)->GetName();
      std::cout << "urdf file name: " << urdf_file_name << std::endl;
      break;
    }
  }

  if (model_ != NULL) {
    base_link_ = model_->GetLink(kBaseName);
    if (base_link_ == NULL) {
      std::cout << "failed get base link from model!" << std::endl;
      return false;
    }

    for (int i = 0; i < kNumJoints; ++i) {
      joint_list_[i] = model_->GetJoint(kJointNames[i]);
    }
    std::cout << "success init model!" << std::endl;
    return true;
  }

  return false;
}

void WorldControllerPlugin::Reset() { InitModel(); }

}  // namespace gazebo
