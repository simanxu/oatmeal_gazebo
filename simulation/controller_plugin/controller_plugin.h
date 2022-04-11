#ifndef SIMULATION_CONTROLLER_PLUGIN_CONTROLLER_PLUGIN_H_
#define SIMULATION_CONTROLLER_PLUGIN_CONTROLLER_PLUGIN_H_

#include <ros/callback_queue.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <ros/subscriber.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <string>
#include <thread>
#include <vector>

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

 private:
  int iterations_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;

  float* q_des_;
  float* qd_des_;
  float* tau_des_;
};

// 向Gazebo注册本插件
GZ_REGISTER_WORLD_PLUGIN(WorldControllerPlugin);

}  // namespace gazebo

#endif  // SIMULATION_CONTROLLER_PLUGIN_CONTROLLER_PLUGIN_H_
