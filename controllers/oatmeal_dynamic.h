#ifndef OATMEAL_GAZEBO_CONTROLLERS_OATMEAL_DYNAMIC_H_
#define OATMEAL_GAZEBO_CONTROLLERS_OATMEAL_DYNAMIC_H_

#include <map>
#include <memory>

#include "common/estimate_result.h"
#include "third_party/eigen/Eigen/Dense"
#include "third_party/rbdl/include/rbdl/rbdl.h"

namespace oatmeal {

namespace rbdl = RigidBodyDynamics;
namespace rbdl_math = RigidBodyDynamics::Math;

class OatmealDynamic {
 public:
  OatmealDynamic();
  ~OatmealDynamic();

  void InitDynamicModel();

  void InitDynamicMatrix();

  void Clear();

  void UpdateRobotStatus(std::shared_ptr<EstimateResult> est_result);

  void UpdateRobotDynamic();

  void UpdateMassMatrixCRBA(const rbdl_math::VectorNd& q);

  void UpdateNonlinearBiasRNEA(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd);

  void UpdateWheelContactJacobianByConstraints(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd);

  void UpdateWheelContactJacobianByContactPoint(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd);

  rbdl_math::VectorNd GetQ() { return this->q_; }

  rbdl_math::VectorNd GetQDot() { return this->qd_; }

  rbdl_math::MatrixNd GetMassMatrix() { return this->H_; }

  rbdl_math::VectorNd GetGravityBias() { return this->G_; }

  rbdl_math::VectorNd GetCoriolisBias() { return this->C_; }

  rbdl_math::VectorNd GetNonlinearBias() { return this->D_; }

  rbdl_math::MatrixNd GetContactJacobian() { return this->ContactJacobian_; }

  rbdl_math::VectorNd GetContactJacobianDotQDot() { return this->ContactJacobianDotQDot_; }

 private:
  std::unique_ptr<rbdl::Model> robot_;
  std::map<int, int> wheel_index_;
  std::map<int, std::string> link_name_;
  rbdl_math::MatrixNd S_;
  rbdl_math::MatrixNd H_;
  rbdl_math::VectorNd D_;
  rbdl_math::VectorNd C_;
  rbdl_math::VectorNd G_;
  rbdl_math::VectorNd q_;
  rbdl_math::VectorNd qd_;
  rbdl_math::VectorNd qdd_;
  rbdl_math::Matrix3d BodyInWorldRotMat_;
  rbdl_math::MatrixNd ContactJacobian_;
  rbdl_math::VectorNd ContactJacobianDotQDot_;
};

}  // namespace oatmeal

#endif  // OATMEAL_GAZEBO_CONTROLLERS_OATMEAL_DYNAMIC_H_
