#ifndef OATMEAL_GAZEBO_CONTROLLERS_OATMEAL_PLANNER_H_
#define OATMEAL_GAZEBO_CONTROLLERS_OATMEAL_PLANNER_H_

#include "third_party/eigen/Eigen/Dense"
#include "third_party/psopt/include/psopt.h"
#include "third_party/qpOASES/include/qpOASES.hpp"
#include "third_party/rbdl/include/rbdl/rbdl.h"

namespace oatmeal {

class OatmealPlanner {
 public:
  OatmealPlanner();
  ~OatmealPlanner();

  void UpdateCommandInput();

  bool CarVelocityControlTask();

  bool PendulumSwingTask();

 private:
  /* data */
};
}  // namespace oatmeal

#endif  // OATMEAL_GAZEBO_CONTROLLERS_OATMEAL_PLANNER_H_
