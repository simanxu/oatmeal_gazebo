#ifndef OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_CONTROLLER_H_
#define OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_CONTROLLER_H_

#include <memory>

#include "common/oatmeal_exchange.h"
#include "controllers/oatmeal_dynamic.h"
#include "controllers/oatmeal_planner.h"
#include "third_party/rbdl/include/rbdl/rbdl.h"

namespace oatmeal {

class OatmealController {
 public:
  OatmealController(std::shared_ptr<ExchangeData> data);

  ~OatmealController();

  void RunController();

  void UpdateState();

  void UpdateCommand();

  Eigen::VectorXd CalculateDynamicWithConstraints(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd,
                                                  const rbdl_math::VectorNd& qdd_task);

 private:
  std::shared_ptr<ExchangeData> data_;
  std::shared_ptr<OatmealDynamic> robot_;
  rbdl_math::MatrixNd S_;
  rbdl_math::MatrixNd H_;
  rbdl_math::VectorNd D_;
  rbdl_math::VectorNd C_;
  rbdl_math::VectorNd G_;
  rbdl_math::VectorNd q_;
  rbdl_math::VectorNd qd_;
  rbdl_math::VectorNd qdd_task_;
  rbdl_math::Matrix3d RotMat_BodyInWorld_;
  rbdl_math::MatrixNd ContactJacobian_;
  rbdl_math::VectorNd ContactJacobianDotQDot_;
};
}  // namespace oatmeal

#endif  // OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_CONTROLLER_H_
