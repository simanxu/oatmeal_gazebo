#include "controller_plugin.h"

namespace {

std::vector<std::string> JointNames = {"Joint_FR", "Joint_FL", "Joint_HR", "Joint_HL"};

std::vector<std::string> ContactNames = {"wheel_FR_collision", "wheel_FL_collision", "wheel_HR_collision",
                                         "wheel_HL_collision"};

}  // namespace

namespace gazebo {

CarWheelPlugin::CarWheelPlugin() {
  joint_count_ = 4;
  q_des_ = new float[joint_count_];
  qd_des_ = new float[joint_count_];
  tau_des_ = new float[joint_count_];
  joint_ = new physics::JointPtr[joint_count_];
  for (int i = 0; i < joint_count_; ++i) {
    q_des_[i] = 0.0;
    qd_des_[i] = 0.0;
    tau_des_[i] = 0.0;
    joint_[i] = NULL;
  }
}

void CarWheelPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
  std::cout << "[CarWheel] plugin name: " << _sdf->Get<std::string>("filename") << std::endl;
  std::cout << "[CarWheel] world name: " << _world->Name() << std::endl;
  world_ = _world;
  iterations_ = 0;

  // init rostopic
  node_handle_.setCallbackQueue(&callback_queue_);
  sub_mode_ = node_handle_.subscribe("/car_mode_data", 10, &CarWheelPlugin::ModeCallback, this);

  // init imu_data
  imu_data_.quat = {1, 0, 0, 0};
  imu_data_.gyro.setZero();
  imu_data_.acc << 0, 0, 9.8;

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateEnd(std::bind(&CarWheelPlugin::OnUpdateEnd, this));
}

void CarWheelPlugin::OnUpdateEnd() {
  if (!model_) InitModel();

  callback_queue_.callAvailable();

  // Update IMU sensor
  LoadImuData();
  // fprintf(stderr, "IMU acc %3.5f %3.5f %3.5f\n", imu_data_.acc(0), imu_data_.acc(1),
  //         imu_data_.acc(2));

  CarController();
  ++iterations_;
  // fprintf(stderr, "current iterations %d\n", iterations_);
}

void CarWheelPlugin::LoadImuData() {
  auto rot = model_->WorldPose().Rot();
  rot.Normalize();
  auto lin_vel = model_->WorldLinearVel();
  auto gravity = world_->Gravity();
  auto lin_acc = rot.RotateVectorReverse(model_->WorldLinearAccel() - gravity);
  auto ang_vel = rot.RotateVectorReverse(model_->WorldAngularVel());

  imu_data_.quat = {
      static_cast<float>(rot.W()),
      static_cast<float>(rot.X()),
      static_cast<float>(rot.Y()),
      static_cast<float>(rot.Z()),
  };
  imu_data_.gyro = {
      static_cast<float>(ang_vel.X()),
      static_cast<float>(ang_vel.Y()),
      static_cast<float>(ang_vel.Z()),
  };
  imu_data_.acc = {
      static_cast<float>(lin_acc.X()),
      static_cast<float>(lin_acc.Y()),
      static_cast<float>(lin_acc.Z()),
  };
}

void CarWheelPlugin::InitModel() {
  // Gazebo world model count
  int model_count = world_->ModelCount();
  // Default Value
  for (int i = 0; i < model_count; ++i) {
    if (world_->ModelByIndex(i)->GetJointCount() > 0) {
      // ground plane dof == 0
      model_ = world_->ModelByIndex(i);
      std::cout << "[CarWheel] model name: " << world_->ModelByIndex(i)->GetName()
                << ", joint count: " << model_->GetJointCount() << std::endl;
      break;
    }
    if (i == model_count - 1) {
      std::cerr << "We cannot found proper model, exit plugin!" << std::endl;
      return;
    }
  }
  if (model_ != NULL) {
    for (int i = 0; i < joint_count_; ++i) {
      joint_[i] = model_->GetJoint(JointNames[i]);
    }
  }
  std::cout << "[CarWheel] initialized!" << std::endl;
}

void CarWheelPlugin::CarController() {
  for (int i = 0; i < 4; ++i) {
    auto joint = model_->GetJoint(JointNames[i]);
    if (control_mode_ == 1) {
      joint->SetForce(0, 1);
    } else {
      joint->SetForce(0, 0);
    }
  }
}

void CarWheelPlugin::ModeCallback(const std_msgs::Int64& msg) {
  int mode = (msg.data >= 10) ? 10 : msg.data;
  std::cout << "Control Mode = " << mode << std::endl;
  control_mode_ = mode;
}

void CarWheelPlugin::Reset() { InitModel(); }

}  // namespace gazebo
