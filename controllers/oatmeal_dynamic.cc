#include "controllers/oatmeal_dynamic.h"

#include <chrono>
#include <iostream>

#include "common/math_tools.h"
#include "common/oatmeal_constants.h"

namespace oatmeal {

OatmealDynamic::OatmealDynamic() {
  this->InitDynamicModel();
  this->InitDynamicMatrix();
}

OatmealDynamic::~OatmealDynamic() {}

void OatmealDynamic::Clear() { this->InitDynamicMatrix(); }

void OatmealDynamic::UpdateRobotStatus(std::shared_ptr<EstimateResult> est_result) {
  q_.head(kThreeDims) = est_result->base_pos;
  q_.segment(kThreeDims, kThreeDims) = est_result->base_quat.vec();
  q_(robot_->qdot_size) = est_result->base_quat.w();
  q_.segment(kSixDims, kNumJoints) = est_result->status.q;

  qd_.head(kThreeDims) = est_result->base_vel_world;
  qd_.segment(kThreeDims, kThreeDims) = est_result->base_omega_body;
  qd_.segment(kSixDims, kNumJoints) = est_result->status.qd;

  BodyInWorldRotMat_ = est_result->base_rot_mat;
}

void OatmealDynamic::UpdateRobotDynamic() {
  this->UpdateMassMatrixCRBA(q_);
  this->UpdateNonlinearBiasRNEA(q_, qd_);
  this->UpdateWheelContactJacobianByContactPoint(q_, qd_);
}

// Calculate Mass Matrix using Composite-Rigid-Body-Algorithm(CRBA)
// If UpdateKinematicsCustom first, update_kinematics in CRBA can set to false
// Else must set update_kinematics to true
// This function only evaluates the entries of H that are non-zero. One Before calling this function one has to ensure
// that all other values have been set to zero, e.g. by calling H.setZero().
void OatmealDynamic::UpdateMassMatrixCRBA(const rbdl_math::VectorNd& q) {
  H_.setZero();
  rbdl::UpdateKinematicsCustom(*robot_, &q, NULL, NULL);
  rbdl::CompositeRigidBodyAlgorithm(*robot_, q, H_, false);
  // rbdl::CompositeRigidBodyAlgorithm(*robot_, q, H_, true);
}

// Calculate Nonlinear Matrix using Recursive Newton-Euler Algorithm(RNEA)
// Calculate Gravity first and Nonlinear Effect - Gravity = Coriolis
void OatmealDynamic::UpdateNonlinearBiasRNEA(const rbdl_math::VectorNd& q, const rbdl_math::VectorNd& qd) {
  rbdl_math::VectorNd vec_zero = rbdl_math::VectorNd::Zero(kNumDofs);
  rbdl::UpdateKinematicsCustom(*robot_, &q, &qd, &vec_zero);
  G_.setZero();
  rbdl::NonlinearEffects(*robot_, q, vec_zero, G_);
  D_.setZero();
  rbdl::NonlinearEffects(*robot_, q, qd, D_);
  C_ = D_ - G_;
}

// Wheel's are suffer with nonholonomic constraints
// rbdl::CalcPointJacobian6D
// the first 3 rows are rotational
// the last 3 rows are translational
// Computational time for four wheels 1.42ms
void OatmealDynamic::UpdateWheelContactJacobianByConstraints(const rbdl_math::VectorNd& q,
                                                             const rbdl_math::VectorNd& qd) {
  // auto tic = std::chrono::system_clock::now();
  for (unsigned int wheel : {kWheelFR, kWheelFL, kWheelHR, kWheelHL}) {
    // Calculate Jacobian of the center of the wheel
    rbdl_math::MatrixNd JacobianWheel6D = rbdl_math::MatrixNd::Zero(kSixDims, kNumDofs);
    int link_id = robot_->GetBodyId(link_name_[wheel].c_str());
    rbdl::CalcPointJacobian6D(*robot_, q, link_id, rbdl_math::Vector3dZero, JacobianWheel6D);

    // Calculate JacobianDot * QDot of the center of the wheel
    rbdl_math::VectorNd qdd_zero = rbdl_math::VectorNd::Zero(kNumDofs);
    rbdl_math::SpatialVector JacobianWheelDotQDot6D = rbdl_math::SpatialVector::Zero();
    JacobianWheelDotQDot6D = rbdl::CalcPointAcceleration6D(*robot_, q, qd, qdd_zero, link_id, rbdl_math::Vector3dZero);
    rbdl_math::Vector3d JacobianWheelDotQDot = JacobianWheelDotQDot6D.tail(kThreeDims);

    // Calculate vec(y_w)
    rbdl_math::Vector3d yw = BodyInWorldRotMat_ * rbdl_math::Vector3d::UnitY();
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
}

// Computational time for four wheels 1.35ms
void OatmealDynamic::UpdateWheelContactJacobianByContactPoint(const rbdl_math::VectorNd& q,
                                                              const rbdl_math::VectorNd& qd) {
  // auto tic = std::chrono::system_clock::now();
  for (unsigned int wheel : {kWheelFR, kWheelFL, kWheelHR, kWheelHL}) {
    // Calculate contact point position wrt wheel cooridinate
    double theta = q(wheel_index_[wheel] + 6);
    rbdl_math::Vector3d contact_point(kWheelRadius * std::sin(theta), 0.0, -kWheelRadius * std::cos(theta));

    // Calculate Jacobian of the contact point of the wheel
    int link_id = robot_->GetBodyId(link_name_[wheel].c_str());
    rbdl_math::MatrixNd JacobianContact = rbdl_math::MatrixNd::Zero(kThreeDims, kNumDofs);
    rbdl::CalcPointJacobian(*robot_, q, link_id, contact_point, JacobianContact);

    // Calculate JacobianDot * QDot of the contact point of the wheel
    rbdl_math::VectorNd qdd_zero = rbdl_math::VectorNd::Zero(kNumDofs);
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
}

void OatmealDynamic::InitDynamicModel() {
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

  if (robot_->dof_count != kNumDofs) {
    std::cerr << "failed init robot dynamic model" << std::endl;
  }
}

void OatmealDynamic::InitDynamicMatrix() {
  S_.setZero(kNumJoints, kNumDofs);
  S_.rightCols(kNumJoints).setIdentity();
  H_.setZero(kNumDofs, kNumDofs);
  D_.setZero(kNumDofs);
  C_.setZero(kNumDofs);
  G_.setZero(kNumDofs);
  q_.setZero(robot_->q_size);
  qd_.setZero(kNumDofs);
  qdd_.setZero(kNumDofs);
  ContactJacobian_.setZero(kNumConstraintDimsPerWheel * kNumWheels, kNumDofs);
  ContactJacobianDotQDot_.setZero(kNumConstraintDimsPerWheel * kNumWheels);
}
}  // namespace oatmeal
