#include <cmath>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ugv_localization_mvp/transform_math.hpp"

namespace
{
constexpr double kTolerance = 1.0e-9;

TEST(TransformMath, AppliesInverseBaseToLidarExtrinsic)
{
  geometry_msgs::msg::Pose raw;
  raw.position.x = 11.0;
  raw.position.y = 2.0;
  raw.orientation.w = 1.0;
  const auto extrinsic = ugv_localization_mvp::makeTransform(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const auto result = ugv_localization_mvp::odomBaseFromRawLidar(raw, extrinsic);
  EXPECT_NEAR(result.translation.x, 10.0, kTolerance);
  EXPECT_NEAR(result.translation.y, 2.0, kTolerance);
}

TEST(TransformMath, ComputesMapOdomFromPathAndOdomStart)
{
  geometry_msgs::msg::Pose odom_base;
  odom_base.position.x = 2.0;
  odom_base.position.y = 1.0;
  odom_base.orientation.w = 1.0;
  const auto result = ugv_localization_mvp::mapOdomFromStart(5.0, 4.0, 0.0, 0.0, odom_base);
  EXPECT_NEAR(result.translation.x, 3.0, kTolerance);
  EXPECT_NEAR(result.translation.y, 3.0, kTolerance);
}

TEST(TransformMath, PreservesRequestedPathHeading)
{
  geometry_msgs::msg::Pose odom_base;
  odom_base.orientation.w = 1.0;
  const auto result = ugv_localization_mvp::mapOdomFromStart(0.0, 0.0, 0.0, 1.2, odom_base);
  tf2::Quaternion q;
  tf2::fromMsg(result.rotation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  EXPECT_NEAR(yaw, 1.2, kTolerance);
}
}  // namespace
