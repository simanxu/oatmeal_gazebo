#ifndef OATMEAL_GAZEBO_COMMON_OATMEAL_CONSTANTS_H_
#define OATMEAL_GAZEBO_COMMON_OATMEAL_CONSTANTS_H_

namespace oatmeal {

enum LinkList {
  kBody = 0,
  kWheelFR = 1,
  kWheelFL = 2,
  kWheelHR = 3,
  kWheelHL = 4,
  kPendulum = 5,
};

constexpr int kThreeDims = 3;
constexpr int kSixDims = 6;

constexpr int kNumDofs = 11;
constexpr int kNumWheels = 4;
constexpr int kNumLinks = 6;
constexpr int kNumJoints = 5;

constexpr int kNumConstraintDimsPerWheel = 2;

constexpr double kGravity = 9.81;
constexpr double kFrictionCoefs = 0.5;

constexpr double kWheelRadius = 0.051;
constexpr double kBodyFrontLength = 0.095;
constexpr double kBodyHindLength = 0.115;
constexpr double kBodyWidthLength = 0.08;

}  // namespace oatmeal

#endif  // OATMEAL_GAZEBO_COMMON_OATMEAL_CONSTANTS_H_
