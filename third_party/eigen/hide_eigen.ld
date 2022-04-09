{
  global: *;
  local:
    extern "C++" {
      Eigen::*;
      *Eigen::internal::*;
      *Eigen::MatrixBase*;
      *Eigen::HouseholderSequence*;
      *Eigen::ColPivHouseholderQR*;
      *Eigen::SVDBase*;
      *Eigen::HouseholderQR*;
    };
};
