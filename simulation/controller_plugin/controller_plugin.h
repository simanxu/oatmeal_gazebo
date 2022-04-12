#ifndef OATMEAL_GAZEBO_SIMULATION_CONTROLLER_PLUGIN_CONTROLLER_PLUGIN_H_
#define OATMEAL_GAZEBO_SIMULATION_CONTROLLER_PLUGIN_CONTROLLER_PLUGIN_H_

#include <ros/callback_queue.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <algorithm>
#include <string>
#include <thread>
#include <vector>

#include "controllers/io_exchange_data.h"
#include "third_party/eigen/Eigen/Dense"
#include "third_party/rbdl/include/rbdl/rbdl.h"

namespace rbdl = RigidBodyDynamics;
namespace rbdl_math = RigidBodyDynamics::Math;

namespace gazebo {
class WorldControllerPlugin : public WorldPlugin {
 public:
  WorldControllerPlugin();
  ~WorldControllerPlugin() = default;

 public:
  // 在插入本插件的时候，Gazebo就会调用Load()函数。
  // 该函数是一个虚函数，我们可以通过重载该函数来对插件进行一些初始化的操作。
  // 输入参数_model是一个指向与本插件相关联的模型的指针。
  // 输入参数_sdf则是一个指向本插件SDF元素的指针。
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  void Reset();

  // 从World中载入Model
  bool InitModel();

  // Called by the world update end event
  void OnUpdateEnd();

 private:
  void GeJoyStateCb(const sensor_msgs::Joy::ConstPtr& msgIn);

  void RosThreadForCmdMsg();

  void RosThreadForLogMsg();

  void UpdateSensorsStatus();

  void UpdatePositionController();

  void UpdateVelocityController();

  void UpdateTorqueController();

 private:
  int iterations_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::JointPtr* joint_list_;

  physics::LinkPtr base_link_;

  Eigen::Quaterniond base_quat_act_;
  Eigen::Vector3d base_xyz_act_;
  Eigen::Vector3d base_rpy_act_;
  Eigen::Vector3d base_linear_vel_act_;
  Eigen::Vector3d base_linear_acc_act_;
  Eigen::Vector3d base_angular_vel_act_;

  double* q_des_;
  double* qd_des_;
  double* tau_des_;

  double* q_act_;
  double* qd_act_;
  double* tau_act_;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;

  ros::NodeHandle node_handle_;
  // Ros thread for cmd
  std::thread* cmd_ros_thread_;
  ros::Subscriber joy_stick_sub_;
  ros::CallbackQueue callback_queue_;
  // Ros thread for pub data
  std::thread* pub_ros_thread_;
  ros::Publisher data_publisher_;

  // Joystick
  JoystickCMD sim_joy_cmd_;
  bool button_A_release_;
  bool button_B_release_;
  bool button_X_release_;
  bool button_Y_release_;
  bool button_LB_release_;
  bool button_RB_release_;

  // Control mode
  unsigned int control_mode_;
};

// 向Gazebo注册本插件
GZ_REGISTER_WORLD_PLUGIN(WorldControllerPlugin);
}  // namespace gazebo

#endif  // OATMEAL_GAZEBO_SIMULATION_CONTROLLER_PLUGIN_CONTROLLER_PLUGIN_H_
