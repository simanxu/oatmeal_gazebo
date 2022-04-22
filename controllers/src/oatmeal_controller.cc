#include "controllers/src/oatmeal_controller.h"

#include <iostream>

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

bool OatmealController::Test() {
  std::cout << "Hello Test" << std::endl;
  return true;
}

void OatmealController::RunController(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd,
                                      const rbdl_math::VectorNd& qdd_des, rbdl_math::VectorNd& torque_command) {
  this->UpdateMassMatrixCRBA(q);
  this->UpdateNonlinearBiasRNEA(q, qd);
  this->UpdateWheelContactJacobian(q, qd);
  torque_command = this->CalculateInverseDynamic(q, qd, qdd_des);
}

rbdl_math::VectorNd OatmealController::CalculateInverseDynamic(const rbdl_math::VectorNd& q,
                                                               const rbdl_math::VectorNd& qd,
                                                               const rbdl_math::VectorNd& qdd_des) {
  rbdl_math::VectorNd tau(robot_dof_);
  // Update constraints Jacobian
  rbdl::InverseDynamics(*robot_, q, qd, qdd_des, tau);
  return tau;
}

// Calculate Mass Matrix using Composite-Rigid-Body-Algorithm(CRBA)
// If UpdateKinematicsCustom first, update_kinematics in CRBA can set to false
// Else must set update_kinematics to true
void OatmealController::UpdateMassMatrixCRBA(const rbdl_math::VectorNd& q) {
  // rbdl::UpdateKinematicsCustom(*robot_, &q, NULL, NULL);
  // rbdl::CompositeRigidBodyAlgorithm(*robot_, q, H_, false);
  rbdl::CompositeRigidBodyAlgorithm(*robot_, q, H_, true);
}

// Calculate Nonlinear Matrix using Recursive Newton-Euler Algorithm(RNEA)
// Calculate Gravity first and Nonlinear Effect - Gravity = Coriolis
void OatmealController::UpdateNonlinearBiasRNEA(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd) {
  rbdl::UpdateKinematicsCustom(*robot_, &q, &qd, NULL);
  rbdl::NonlinearEffects(*robot_, q, qd, D_);
  rbdl_math::VectorNd qd_zero = rbdl_math::VectorNd::Zero(robot_dof_);
  rbdl::NonlinearEffects(*robot_, q, qd_zero, G_);
  C_ = D_ - G_;
}

// Wheel's are suffer with nonholonomic constraints
void OatmealController::UpdateWheelContactJacobian(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd) {
  for (unsigned int wheel : {kWheelFR, kWheelFL, kWheelHR, kWheelHL}) {
    // Calculate Jacobian of the center of the wheel
    rbdl_math::MatrixNd JacobianWheel(kJacobianDims, robot_dof_);
    int link_id = robot_->GetBodyId(link_name_[wheel].c_str());
    rbdl::CalcPointJacobian(*robot_, q, link_id, rbdl_math::Vector3dZero, JacobianWheel);
    // Calculate JacobianDot * QDot of the center of the wheel
    rbdl_math::VectorNd qdd_zero = rbdl_math::VectorNd::Zero(robot_dof_);
    rbdl_math::SpatialVector JDotQDot6D =
        rbdl::CalcPointAcceleration6D(*robot_, q, qd, qdd_zero, link_id, rbdl_math::Vector3dZero);
    rbdl_math::Vector3d JacobianDotQDotWheel = JDotQDot6D.tail(kJacobianDims);
    // Calculate RotMat of wheel with respect to global
    rbdl_math::Quaternion Quat_BodyInWorld;
    Quat_BodyInWorld.head(3) = q.segment(3, 3);
    Quat_BodyInWorld.tail(1) = q.tail(1);
    RotMat_BodyInWorld_ = Quat_BodyInWorld.toMatrix();
    // Calculate mat(S_w)
    rbdl_math::MatrixNd Sw = rbdl_math::MatrixNd::Zero(robot_dof_, robot_dof_);
    // 这里不是很严谨，因为枚举值不应参与计算，但还没想到更优雅的办法
    Sw(wheel + 5, wheel + 5) = 1.0;
    // Calculate vec(y_w)
    rbdl_math::Vector3d yw = RotMat_BodyInWorld_ * rbdl_math::Vector3d::UnitY();
    // Calculate vec(r_wc)
    rbdl_math::Vector3d rwc = yw.cross(yw.cross(rbdl_math::Vector3d::UnitZ()));
    // Calculate contact point Jacobian and JacobianDotQDot
    rbdl_math::MatrixNd JacobianContact(kJacobianDims, robot_dof_);
    rbdl_math::Matrix3d skew_yw;
    Skew(yw, skew_yw);
    JacobianContact = JacobianWheel + skew_yw * rwc * Sw;
    rbdl_math::MatrixNd JacobianDotQDotContact(kJacobianDims, robot_dof_);
    JacobianDotQDotContact = -JacobianDotQDotWheel - skew_yw * skew_yw * rwc * (Sw * qd) * (Sw * qd);
  }
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
  H_.setZero(robot_dof_, robot_dof_);
  D_.setZero(robot_dof_);
  C_.setZero(robot_dof_);
  G_.setZero(robot_dof_);
  q_.setZero(robot_dof_ + 1);
  qd_.setZero(robot_dof_);
  qdd_.setZero(robot_dof_);
  WheelFRJacobian_.setZero(kJacobianDims, robot_dof_);
  WheelFLJacobian_.setZero(kJacobianDims, robot_dof_);
  WheelHRJacobian_.setZero(kJacobianDims, robot_dof_);
  WheelHLJacobian_.setZero(kJacobianDims, robot_dof_);
  WheelFRJdQd_.setZero(kJacobianDims, robot_dof_);
  WheelFLJdQd_.setZero(kJacobianDims, robot_dof_);
  WheelHRJdQd_.setZero(kJacobianDims, robot_dof_);
  WheelHLJdQd_.setZero(kJacobianDims, robot_dof_);
  return true;
}
