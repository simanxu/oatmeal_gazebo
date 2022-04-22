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

constexpr int kJacobianDims = 3;
constexpr int kNumLinks = 6;
constexpr int kNumJoints = 5;
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

  // some math
  //    Skew(v) =
  //              0  -v(2)  v(1)
  //             v(2)  0   -v(0)
  //            -v(1) v(0)  0
  void Skew(const rbdl_math::Vector3d& v, rbdl_math::Matrix3d& v_skew) {
    v_skew(0, 0) = 0.0;
    v_skew(0, 1) = -v(2);
    v_skew(0, 2) = v(1);
    v_skew(1, 0) = v(2);
    v_skew(1, 1) = 0.0;
    v_skew(1, 2) = -v(0);
    v_skew(2, 0) = -v(1);
    v_skew(2, 1) = v(0);
    v_skew(2, 2) = 0.0;
  }

  void RunController(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd, const rbdl_math::VectorNd& qdd_des,
                     rbdl_math::VectorNd& torque_command);

  bool InitDynamicModel();
  bool InitDynamicMatrix();

  rbdl_math::VectorNd CalculateInverseDynamic(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd,
                                              const rbdl_math::VectorNd& qdd_des);

  void UpdateMassMatrixCRBA(const rbdl_math::VectorNd& q);
  void UpdateNonlinearBiasRNEA(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd);
  void UpdateWheelContactJacobian(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd);

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
  rbdl_math::MatrixNd WheelFRJacobian_;
  rbdl_math::MatrixNd WheelFRJdQd_;
  rbdl_math::MatrixNd WheelFLJacobian_;
  rbdl_math::MatrixNd WheelFLJdQd_;
  rbdl_math::MatrixNd WheelHRJacobian_;
  rbdl_math::MatrixNd WheelHRJdQd_;
  rbdl_math::MatrixNd WheelHLJacobian_;
  rbdl_math::MatrixNd WheelHLJdQd_;
};

#endif  // OATMEAL_GAZEBO_CONTROLLERS_SRC_OATMEAL_CONTROLLER_H_
