#ifndef OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_CONTROLLER_H_
#define OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_CONTROLLER_H_

#include <map>
#include <memory>

#include "third_party/eigen/Eigen/Dense"
#include "third_party/psopt/include/psopt.h"
#include "third_party/qpOASES/include/qpOASES.hpp"
#include "third_party/rbdl/include/rbdl/rbdl.h"

namespace rbdl = RigidBodyDynamics;
namespace rbdl_math = RigidBodyDynamics::Math;

constexpr int kThreeDims = 3;
constexpr int kSixDims = 6;
constexpr int kNumWheels = 4;
constexpr int kNumLinks = 6;
constexpr int kNumJoints = 5;
// 2: J * qd = 0 (x,z)
// 2: Jd* qd + J * qdd = 0 (x,z)
constexpr int kNumConstraintDimsPerWheel = 2;
constexpr double kWheelRadius = 0.051;
constexpr double kGravity = 9.81;
constexpr double kBodyFrontLength = 0.095;
constexpr double kBodyHindLength = 0.115;
constexpr double kBodyWidthLength = 0.08;

enum LinkList : unsigned int {
  kBody = 0,
  kWheelFR = 1,
  kWheelFL = 2,
  kWheelHR = 3,
  kWheelHL = 4,
  kPendulum,
};

class OatmealController {
 public:
  OatmealController();

  ~OatmealController();

  bool Test();

  void RunController(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd, const rbdl_math::VectorNd& qdd_des,
                     rbdl_math::VectorNd& torque_command);

  bool InitDynamicModel();
  bool InitDynamicMatrix();

  rbdl_math::VectorNd CalculateInverseDynamic(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd,
                                              const rbdl_math::VectorNd& qdd_des);

  rbdl_math::VectorNd CalculateDynamicWithConstraints(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd,
                                                      const rbdl_math::VectorNd& qdd_task);

  void UpdateMassMatrixCRBA(const rbdl_math::VectorNd& q);
  void UpdateNonlinearBiasRNEA(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd);
  void UpdateWheelContactJacobian(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd);
  void UpdateWheelContactJacobianByContactPoint(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd);

  rbdl_math::MatrixNd GetMassMatrix() { return this->H_; }
  rbdl_math::VectorNd GetGravityBias() { return this->G_; }
  rbdl_math::VectorNd GetCoriolisBias() { return this->C_; }
  rbdl_math::VectorNd GetNonlinearBias() { return this->D_; }

 private:
  std::unique_ptr<rbdl::Model> robot_;
  std::map<int, std::string> link_name_;
  int robot_dof_;
  rbdl_math::MatrixNd S_;
  rbdl_math::MatrixNd H_;
  rbdl_math::VectorNd D_;
  rbdl_math::VectorNd C_;
  rbdl_math::VectorNd G_;
  rbdl_math::VectorNd q_;
  rbdl_math::VectorNd qd_;
  rbdl_math::VectorNd qdd_;
  rbdl_math::Matrix3d RotMat_BodyInWorld_;
  rbdl_math::MatrixNd ContactJacobian_;
  rbdl_math::VectorNd ContactJacobianDotQDot_;
};

#endif  // OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_CONTROLLER_H_
