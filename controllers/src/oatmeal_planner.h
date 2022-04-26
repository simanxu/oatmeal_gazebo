#ifndef OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_PLANNER_H_
#define OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_PLANNER_H_

#include "third_party/eigen/Eigen/Dense"
#include "third_party/psopt/include/psopt.h"
#include "third_party/qpOASES/include/qpOASES.hpp"
#include "third_party/rbdl/include/rbdl/rbdl.h"

// TODO
// 1. 手柄指令处理独立到此处
// 2. 先规划，再控制

class OatmealPlanner {
 public:
  OatmealPlanner();
  ~OatmealPlanner();

  bool CarVelocityControlTask();

  bool PendulumSwingTask();

 private:
  /* data */
};

#endif  // OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_PLANNER_H_
