#include "controllers/oatmeal_controller.h"

#include <chrono>
#include <iostream>

#include "common/math_tools.h"
#include "third_party/eigen/Eigen/Dense"
#include "third_party/qpOASES/include/qpOASES.hpp"

namespace oatmeal {

OatmealController::OatmealController(std::shared_ptr<ExchangeData> data) {
  data_ = data;
  robot_ = data->oatmeal;
  S_.setZero(kNumJoints, kNumDofs);
  S_.rightCols(kNumJoints).setIdentity();
  H_.setZero(kNumDofs, kNumDofs);
  D_.setZero(kNumDofs);
  C_.setZero(kNumDofs);
  G_.setZero(kNumDofs);
  q_.setZero(kNumDofs + 1);
  qd_.setZero(kNumDofs);
  qdd_task_.setZero(kNumDofs);
  ContactJacobian_.setZero(kNumConstraintDimsPerWheel * kNumWheels, kNumDofs);
  ContactJacobianDotQDot_.setZero(kNumConstraintDimsPerWheel * kNumWheels);
}

OatmealController::~OatmealController() {}

void OatmealController::RunController() {
  this->UpdateState();
  this->UpdateCommand();
  this->CalculateDynamicWithConstraints(q_, qd_, qdd_task_);
}

void OatmealController::UpdateState() {
  robot_->UpdateRobotStatus(data_->estimate_result);
  robot_->UpdateRobotDynamic();
  q_ = robot_->GetQ();
  qd_ = robot_->GetQDot();
  H_ = robot_->GetMassMatrix();
  G_ = robot_->GetGravityBias();
  C_ = robot_->GetCoriolisBias();
  ContactJacobian_ = robot_->GetContactJacobian();
  ContactJacobianDotQDot_ = robot_->GetContactJacobianDotQDot();
}

void OatmealController::UpdateCommand() {}

Eigen::VectorXd OatmealController::CalculateDynamicWithConstraints(const rbdl_math::VectorNd& q,
                                                                   const rbdl_math::VectorNd& qd,
                                                                   const rbdl_math::VectorNd& qdd_task) {
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(kNumJoints);
  const int kNumVariables = kNumJoints + kNumConstraintDimsPerWheel * kNumWheels;
  const int kNumConstraints = 8;
  qpOASES::QProblem wbc_qp = qpOASES::QProblem(kNumVariables, kNumConstraints);
  qpOASES::Options options;
  options.printLevel = qpOASES::PL_NONE;
  options.initialStatusBounds = qpOASES::ST_INACTIVE;
  options.enableEqualities = qpOASES::BT_TRUE;
  options.enableRegularisation = qpOASES::BT_TRUE;
  wbc_qp.setOptions(options);

  // calculate qdd_des based on J * qdd + Jd * qd = 0
  Eigen::VectorXd qdd_float_base = qdd_task.head(kSixDims);
  Eigen::VectorXd contact_task_acc = Eigen::VectorXd::Zero(kNumConstraintDimsPerWheel * kNumWheels);
  Eigen::VectorXd contact_partial_acc =
      contact_task_acc - ContactJacobianDotQDot_ - ContactJacobian_.leftCols(kSixDims) * qdd_float_base;

  Eigen::MatrixXd contact_partial_Jacobian = ContactJacobian_.rightCols(kNumJoints);
  Eigen::MatrixXd contact_partial_Jacobian_pinv = math::PseudoinverseSvd(contact_partial_Jacobian);
  Eigen::VectorXd qdd_des(kNumDofs);
  qdd_des.head(kSixDims) = qdd_float_base;
  qdd_des.tail(kNumJoints) = contact_partial_Jacobian_pinv * contact_partial_acc;

  // init equality Ax = B
  Eigen::MatrixXd A(kNumDofs, kNumVariables);
  A.leftCols(kNumJoints) = S_.transpose();
  A.rightCols(kNumConstraintDimsPerWheel * kNumWheels) = ContactJacobian_.transpose();
  Eigen::VectorXd B = H_ * qdd_des + C_ + G_;

  // init Hessian(H) and gradient(G), Ax = B
  Eigen::Matrix<double, kNumVariables, kNumVariables, Eigen::RowMajor> Penalty;
  Penalty.setZero();
  Penalty.block(0, 0, kNumJoints, kNumJoints).setIdentity();
  Eigen::Matrix<double, kNumVariables, kNumVariables, Eigen::RowMajor> H;
  H = A.transpose() * A + Penalty;
  Eigen::VectorXd G = -A.transpose() * B;
  Eigen::VectorXd wbc_solve = Eigen::VectorXd::Zero(kNumVariables);

  // init friction cone constraints and boundary constraints
  Eigen::Matrix<double, kNumConstraints, kNumVariables, Eigen::RowMajor> Ac;
  Ac.setZero();
  Eigen::VectorXd lbc = Eigen::VectorXd::Zero(kNumConstraints);
  Eigen::VectorXd ubc = Eigen::VectorXd::Constant(kNumConstraints, 1e8);
  Eigen::VectorXd lbx(kNumVariables);
  Eigen::VectorXd ubx(kNumVariables);
  lbx.segment(0, kNumJoints).array() = -30.0;
  ubx.segment(0, kNumJoints).array() = 30.0;
  for (int i = 0; i < kNumWheels; ++i) {
    int cst_left_id = i * kNumConstraintDimsPerWheel;
    int cst_right_id = i * kNumConstraintDimsPerWheel + 1;
    int fx_id = kNumJoints + i * kNumConstraintDimsPerWheel;
    int fz_id = kNumJoints + i * kNumConstraintDimsPerWheel + 1;

    Ac(cst_left_id, fx_id) = 1.0;              // Fx + uFz > 0
    Ac(cst_left_id, fz_id) = kFrictionCoefs;   // Fx + uFz > 0
    Ac(cst_right_id, fx_id) = -1.0;            // -Fx + uFz > 0
    Ac(cst_right_id, fz_id) = kFrictionCoefs;  // -Fx + uFz > 0

    lbx(fx_id) = -50.0;  // Contact force X
    lbx(fz_id) = 0.0;    // Contact force Z
    ubx(fx_id) = 50.0;   // Contact force X
    ubx(fz_id) = 500.0;  // Contact force Z
  }

  int nWSR = 10000;
  qpOASES::returnValue init_exit = wbc_qp.init(H.data(), G.data(), Ac.data(), lbx.data(), ubx.data(), lbc.data(),
                                               ubc.data(), nWSR, 0, wbc_solve.data(), 0);
  qpOASES::returnValue solve_exit = wbc_qp.getPrimalSolution(wbc_solve.data());
  if (init_exit == qpOASES::SUCCESSFUL_RETURN && solve_exit == qpOASES::SUCCESSFUL_RETURN) {
    tau = wbc_solve.head(kNumJoints);
    std::cout << "WBC: " << nWSR << " " << (A * wbc_solve - B).norm() << std::endl;
    std::cout << "Joint torque: " << wbc_solve.head(kNumJoints).transpose() << std::endl;
    std::cout << "Ground froce: " << wbc_solve.tail(kNumConstraintDimsPerWheel * kNumWheels).transpose() << std::endl;
  } else {
    fprintf(stderr, "Solve wbc problem failed, error code %d, %d.\n", init_exit, solve_exit);
  }
  return tau;
}

}  // namespace oatmeal
