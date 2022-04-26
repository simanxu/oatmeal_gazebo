#include "controllers/src/oatmeal_controller.h"

#include <chrono>
#include <iostream>

#include "controllers/src/math_tools.h"

OatmealController::OatmealController() {
  bool init_ok = InitDynamicModel();
  if (!init_ok) {
    std::cerr << "failed init robot dynamic model" << std::endl;
  }
  init_ok = InitDynamicMatrix();
  if (!init_ok) {
    std::cerr << "failed init robot dynamic matrix" << std::endl;
  }
}

OatmealController::~OatmealController() {}

void OatmealController::RunController(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd,
                                      const rbdl_math::VectorNd& qdd_task, rbdl_math::VectorNd& torque_command) {
  torque_command.setZero();
  this->UpdateMassMatrixCRBA(q);
  this->UpdateNonlinearBiasRNEA(q, qd);
  this->UpdateWheelContactJacobian(q, qd);
  torque_command = this->CalculateDynamicWithConstraints(q, qd, qdd_task);
}

rbdl_math::VectorNd OatmealController::CalculateInverseDynamic(const rbdl_math::VectorNd& q,
                                                               const rbdl_math::VectorNd& qd,
                                                               const rbdl_math::VectorNd& qdd_des) {
  rbdl_math::VectorNd tau = rbdl_math::VectorNd::Zero(robot_dof_);
  rbdl::InverseDynamics(*robot_, q, qd, qdd_des, tau);
  return tau;
}

rbdl_math::VectorNd OatmealController::CalculateDynamicWithConstraints(const rbdl_math::VectorNd& q,
                                                                       const rbdl_math::VectorNd& qd,
                                                                       const rbdl_math::VectorNd& qdd_task) {
  rbdl_math::VectorNd tau = Eigen::VectorXd::Zero(kNumJoints);
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
  Eigen::VectorXd qdd_des(robot_dof_);
  qdd_des.head(kSixDims) = qdd_float_base;
  qdd_des.tail(kNumJoints) = contact_partial_Jacobian_pinv * contact_partial_acc;

  // init equality Ax = B
  Eigen::MatrixXd A(robot_dof_, kNumVariables);
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

// Calculate Mass Matrix using Composite-Rigid-Body-Algorithm(CRBA)
// If UpdateKinematicsCustom first, update_kinematics in CRBA can set to false
// Else must set update_kinematics to true
// This function only evaluates the entries of H that are non-zero. One Before calling this function one has to ensure
// that all other values have been set to zero, e.g. by calling H.setZero().
void OatmealController::UpdateMassMatrixCRBA(const rbdl_math::VectorNd& q) {
  H_.setZero();
  // rbdl::UpdateKinematicsCustom(*robot_, &q, NULL, NULL);
  // rbdl::CompositeRigidBodyAlgorithm(*robot_, q, H_, false);
  rbdl::CompositeRigidBodyAlgorithm(*robot_, q, H_, true);
  // std::cout << "Update mass matrix H pass" << std::endl;
}

// Calculate Nonlinear Matrix using Recursive Newton-Euler Algorithm(RNEA)
// Calculate Gravity first and Nonlinear Effect - Gravity = Coriolis
void OatmealController::UpdateNonlinearBiasRNEA(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd) {
  rbdl_math::VectorNd vec_zero = rbdl_math::VectorNd::Zero(robot_dof_);
  rbdl::UpdateKinematicsCustom(*robot_, &q, &qd, &vec_zero);
  G_.setZero();
  rbdl::NonlinearEffects(*robot_, q, vec_zero, G_);
  D_.setZero();
  rbdl::NonlinearEffects(*robot_, q, qd, D_);
  C_ = D_ - G_;
  // std::cout << "Update nonlinear bias C, G pass" << std::endl;
}

// Wheel's are suffer with nonholonomic constraints
// rbdl::CalcPointJacobian6D
// the first 3 rows are rotational
// the last 3 rows are translational
// Computational time for four wheels 1.42ms
void OatmealController::UpdateWheelContactJacobian(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd) {
  // auto tic = std::chrono::system_clock::now();
  for (unsigned int wheel : {kWheelFR, kWheelFL, kWheelHR, kWheelHL}) {
    // Calculate Jacobian of the center of the wheel
    rbdl_math::MatrixNd JacobianWheel6D = rbdl_math::MatrixNd::Zero(kSixDims, robot_dof_);
    int link_id = robot_->GetBodyId(link_name_[wheel].c_str());
    rbdl::CalcPointJacobian6D(*robot_, q, link_id, rbdl_math::Vector3dZero, JacobianWheel6D);

    // Calculate JacobianDot * QDot of the center of the wheel
    rbdl_math::VectorNd qdd_zero = rbdl_math::VectorNd::Zero(robot_dof_);
    rbdl_math::SpatialVector JacobianWheelDotQDot6D = rbdl_math::SpatialVector::Zero();
    JacobianWheelDotQDot6D = rbdl::CalcPointAcceleration6D(*robot_, q, qd, qdd_zero, link_id, rbdl_math::Vector3dZero);
    rbdl_math::Vector3d JacobianWheelDotQDot = JacobianWheelDotQDot6D.tail(kThreeDims);

    // Calculate RotMat of wheel with respect to global
    rbdl_math::Quaternion Quat_BodyInWorld;
    Quat_BodyInWorld.head(3) = q.segment(3, 3);
    Quat_BodyInWorld.tail(1) = q.tail(1);
    RotMat_BodyInWorld_ = Quat_BodyInWorld.toMatrix();

    // Calculate vec(y_w)
    rbdl_math::Vector3d yw = RotMat_BodyInWorld_ * rbdl_math::Vector3d::UnitY();
    // Calculate vec(r_wc)
    rbdl_math::Vector3d rwc = kWheelRadius * yw.cross(yw.cross(rbdl_math::Vector3d::UnitZ()));
    // Calculate contact point Jacobian and JacobianDotQDot
    rbdl_math::Matrix3d skew_rwc = math::SkewMatrix(rwc);
    rbdl_math::MatrixNd JacobianContact =
        JacobianWheel6D.bottomRows(kThreeDims) - skew_rwc * JacobianWheel6D.topRows(kThreeDims);

    rbdl_math::Vector3d OmegaWheel = JacobianWheel6D.topRows(kThreeDims) * qd;
    rbdl_math::Matrix3d skew_OmegaWheel = math::SkewMatrix(OmegaWheel);
    rbdl_math::Vector3d JacobianContactDotQDot =
        JacobianWheelDotQDot6D.bottomRows(kThreeDims) + skew_OmegaWheel * skew_OmegaWheel * rwc;

    int nrows = wheel_index_[wheel] * kNumConstraintDimsPerWheel;
    ContactJacobian_.row(nrows) = JacobianContact.row(0);             // x
    ContactJacobian_.row(nrows + 1) = JacobianContact.row(2);         // z
    ContactJacobianDotQDot_(nrows) = JacobianContactDotQDot.x();      // x
    ContactJacobianDotQDot_(nrows + 1) = JacobianContactDotQDot.z();  // z
  }
  // auto toc = std::chrono::system_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic);
  // double time = static_cast<double>(duration.count() * 1000) * std::chrono::microseconds::period::num /
  //               std::chrono::microseconds::period::den;
  // std::cout << "Time consume through wheel Jacobian: " << time << std::endl;
  // std::cout << "Update contact Jacobian matrix J pass" << std::endl;
}

// Computational time for four wheels 1.35ms
void OatmealController::UpdateWheelContactJacobianByContactPoint(const rbdl_math::VectorNd& q,
                                                                 const rbdl_math::VectorNd& qd) {
  // auto tic = std::chrono::system_clock::now();
  for (unsigned int wheel : {kWheelFR, kWheelFL, kWheelHR, kWheelHL}) {
    // Calculate contact point position wrt wheel cooridinate
    double theta = q(wheel_index_[wheel] + 6);
    rbdl_math::Vector3d contact_point(kWheelRadius * std::sin(theta), 0.0, -kWheelRadius * std::cos(theta));

    // Calculate Jacobian of the contact point of the wheel
    int link_id = robot_->GetBodyId(link_name_[wheel].c_str());
    rbdl_math::MatrixNd JacobianContact = rbdl_math::MatrixNd::Zero(kThreeDims, robot_dof_);
    rbdl::CalcPointJacobian(*robot_, q, link_id, contact_point, JacobianContact);

    // Calculate JacobianDot * QDot of the contact point of the wheel
    rbdl_math::VectorNd qdd_zero = rbdl_math::VectorNd::Zero(robot_dof_);
    rbdl_math::SpatialVector JacobianWheelDotQDot6D = rbdl_math::SpatialVector::Zero();
    JacobianWheelDotQDot6D = rbdl::CalcPointAcceleration6D(*robot_, q, qd, qdd_zero, link_id, contact_point);
    rbdl_math::Vector3d JacobianContactDotQDot = JacobianWheelDotQDot6D.tail(kThreeDims);

    int nrows = wheel_index_[wheel] * kNumConstraintDimsPerWheel;
    ContactJacobian_.row(nrows) = JacobianContact.row(0);             // x
    ContactJacobian_.row(nrows + 1) = JacobianContact.row(2);         // z
    ContactJacobianDotQDot_(nrows) = JacobianContactDotQDot.x();      // x
    ContactJacobianDotQDot_(nrows + 1) = JacobianContactDotQDot.z();  // z
  }
  // auto toc = std::chrono::system_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic);
  // double time = static_cast<double>(duration.count() * 1000) * std::chrono::microseconds::period::num /
  //               std::chrono::microseconds::period::den;
  // std::cout << "Time consume by contact point: " << time << std::endl;
  // std::cout << "Update contact Jacobian matrix J pass" << std::endl;
}

bool OatmealController::InitDynamicModel() {
  std::cout << "use default params to init rbdl model" << std::endl;
  robot_ = std::make_unique<rbdl::Model>();
  robot_->gravity = rbdl_math::Vector3d(0.0, 0.0, -kGravity);

  // example = rbdl::Body(mass, rbdl_math::Vector3d(rx, ry, rz),
  //                      rbdl_math::Matrix3d(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz));
  std::vector<unsigned int> link_id(kNumLinks);
  rbdl::Body body_link(5.0, rbdl_math::Vector3d(0.0, 0.0, 0.0), rbdl_math::Vector3d(0.007, 0.02, 0.028));
  rbdl::Body wheel_fr_link(0.05, rbdl_math::Vector3d(0.0, 0.0, 0.0), rbdl_math::Vector3d(1.97E-05, 3.86E-05, 1.97E-05));
  rbdl::Body wheel_fl_link(0.05, rbdl_math::Vector3d(0.0, 0.0, 0.0), rbdl_math::Vector3d(1.97E-05, 3.86E-05, 1.97E-05));
  rbdl::Body wheel_hr_link(0.05, rbdl_math::Vector3d(0.0, 0.0, 0.0), rbdl_math::Vector3d(1.97E-05, 3.86E-05, 1.97E-05));
  rbdl::Body wheel_hl_link(0.05, rbdl_math::Vector3d(0.0, 0.0, 0.0), rbdl_math::Vector3d(1.97E-05, 3.86E-05, 1.97E-05));
  rbdl::Body pendulum_link(0.5, rbdl_math::Vector3d(0.0, 0.0, -0.25),
                           rbdl_math::Vector3d(1.97E-05, 3.86E-05, 1.97E-05));

  rbdl::Joint float_joint(rbdl::JointTypeFloatingBase);
  rbdl::Joint ry_joint(rbdl::JointTypeRevoluteY);

  Eigen::Matrix3d Identity_3_3 = Eigen::Matrix3d::Identity();
  rbdl_math::SpatialTransform flotaing_2_body = rbdl_math::Xtrans(rbdl_math::Vector3dZero);
  rbdl_math::SpatialTransform body_2_fr_wheel(Identity_3_3, Eigen::Vector3d(kBodyFrontLength, -kBodyWidthLength, 0.0));
  rbdl_math::SpatialTransform body_2_fl_wheel(Identity_3_3, Eigen::Vector3d(kBodyFrontLength, kBodyWidthLength, 0.0));
  rbdl_math::SpatialTransform body_2_hr_wheel(Identity_3_3, Eigen::Vector3d(-kBodyHindLength, -kBodyWidthLength, 0.0));
  rbdl_math::SpatialTransform body_2_hl_wheel(Identity_3_3, Eigen::Vector3d(-kBodyHindLength, kBodyWidthLength, 0.0));
  rbdl_math::SpatialTransform body_2_pendulum(Identity_3_3, Eigen::Vector3d(0.0, 0.0, 0.0));

  link_name_.insert(std::pair<int, std::string>(kBody, "Body"));
  link_name_.insert(std::pair<int, std::string>(kWheelFR, "WheelFR"));
  link_name_.insert(std::pair<int, std::string>(kWheelFL, "WheelFL"));
  link_name_.insert(std::pair<int, std::string>(kWheelHR, "WheelHR"));
  link_name_.insert(std::pair<int, std::string>(kWheelHL, "WheelHL"));
  link_name_.insert(std::pair<int, std::string>(kPendulum, "Pendulum"));

  wheel_index_.insert(std::pair<int, int>(kWheelFR, 0));
  wheel_index_.insert(std::pair<int, int>(kWheelFL, 1));
  wheel_index_.insert(std::pair<int, int>(kWheelHR, 2));
  wheel_index_.insert(std::pair<int, int>(kWheelHL, 3));

  link_id[kBody] = robot_->AddBody(0, flotaing_2_body, float_joint, body_link, link_name_[kBody]);
  link_id[kWheelFR] = robot_->AddBody(link_id[kBody], body_2_fr_wheel, ry_joint, wheel_fr_link, link_name_[kWheelFR]);
  link_id[kWheelFL] = robot_->AddBody(link_id[kBody], body_2_fl_wheel, ry_joint, wheel_fl_link, link_name_[kWheelFL]);
  link_id[kWheelHR] = robot_->AddBody(link_id[kBody], body_2_hr_wheel, ry_joint, wheel_hr_link, link_name_[kWheelHR]);
  link_id[kWheelHL] = robot_->AddBody(link_id[kBody], body_2_hl_wheel, ry_joint, wheel_hl_link, link_name_[kWheelHL]);
  link_id[kPendulum] = robot_->AddBody(link_id[kBody], body_2_pendulum, ry_joint, pendulum_link, link_name_[kPendulum]);
  return true;
}

bool OatmealController::InitDynamicMatrix() {
  robot_dof_ = robot_->dof_count;
  S_.setZero(kNumJoints, robot_dof_);
  S_.rightCols(kNumJoints).setIdentity();
  H_.setZero(robot_dof_, robot_dof_);
  D_.setZero(robot_dof_);
  C_.setZero(robot_dof_);
  G_.setZero(robot_dof_);
  q_.setZero(robot_->q_size);
  qd_.setZero(robot_dof_);
  qdd_.setZero(robot_dof_);
  ContactJacobian_.setZero(kNumConstraintDimsPerWheel * kNumWheels, robot_dof_);
  ContactJacobianDotQDot_.setZero(kNumConstraintDimsPerWheel * kNumWheels);
  return true;
}
