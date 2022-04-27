#ifndef OATMEAL_GAZEBO_CONTROLLERS_SRC_MATH_TOOLS_H_
#define OATMEAL_GAZEBO_CONTROLLERS_SRC_MATH_TOOLS_H_

#include "third_party/eigen/Eigen/Dense"

namespace math {

template <typename T>
using RemoveCvref = std::remove_cv_t<std::remove_reference_t<T>>;

template <typename EigenExpression>
using EigenConcreteType = RemoveCvref<typename RemoveCvref<EigenExpression>::EvalReturnType>;

Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d skew_matrix;
  skew_matrix(0, 0) = 0.0;
  skew_matrix(0, 1) = -vec(2);
  skew_matrix(0, 2) = vec(1);
  skew_matrix(1, 0) = vec(2);
  skew_matrix(1, 1) = 0.0;
  skew_matrix(1, 2) = -vec(0);
  skew_matrix(2, 0) = -vec(1);
  skew_matrix(2, 1) = vec(0);
  skew_matrix(2, 2) = 0.0;
  return skew_matrix;
}

// There are three main methods to find the pseudo-inverse:
// * Direct derivative: A = (Aᵀ A)⁻¹ Aᵀ
// * QR decomposition: A = Q R, A⁻¹ = R⁻¹ Qᵀ
// * SVD decomposition: A = U S Vᵀ, A⁻¹ = V S⁻¹ Uᵀ
// QR is the fastest but potentially numerically unstable.
// SVD is slower than QR and numerically stable.

// Returns an evaluated matrix of transposed dimensions.
template <typename Derived>
auto PseudoinverseSvd(const Eigen::MatrixBase<Derived>& A) -> EigenConcreteType<decltype(A.transpose())> {
  using Matrix = EigenConcreteType<Derived>;
  using MatrixTransposed = EigenConcreteType<decltype(A.transpose())>;
  Eigen::JacobiSVD<Matrix> A_svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  auto S = A_svd.singularValues();
  MatrixTransposed S_inv = MatrixTransposed::Zero(A.cols(), A.rows());
  // This counts on the fact that singularValues() are always sorted in decreasing order.
  int num_nonzeros = A_svd.nonzeroSingularValues();
  S_inv.diagonal().segment(0, num_nonzeros) = S.segment(0, num_nonzeros).cwiseInverse();
  return A_svd.matrixV() * S_inv * A_svd.matrixU().transpose();
}

// Returns an evaluated matrix of transposed dimensions.
template <typename Derived>
auto PseudoinverseQr(const Eigen::MatrixBase<Derived>& A) -> EigenConcreteType<decltype(A.transpose())> {
  return A.completeOrthogonalDecomposition().pseudoInverse();
}

}  // namespace math

#endif  // OATMEAL_GAZEBO_CONTROLLERS_SRC_MATH_TOOLS_H_
