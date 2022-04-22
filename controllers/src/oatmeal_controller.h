#ifndef OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_CONTROLLER_H_
#define OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_CONTROLLER_H_

#include "third_party/eigen/Eigen/Dense"
#include "third_party/psopt/include/psopt.h"
#include "third_party/qpOASES/include/qpOASES.hpp"
#include "third_party/rbdl/include/rbdl/rbdl.h"

class OatmealController {
 private:
  /* data */
 public:
  OatmealController();
  ~OatmealController();

  bool Test();
};

#endif  // OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_CONTROLLER_H_
