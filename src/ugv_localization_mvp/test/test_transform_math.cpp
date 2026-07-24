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

TEST(TransformMath, ComposesNonTrivialOdomStartBackToRequestedMapStart)
{
  geometry_msgs::msg::Pose odom_base;
  odom_base.position.x = 2.0;
  odom_base.position.y = -1.0;
  odom_base.position.z = 0.3;
  tf2::Quaternion odom_rotation;
  odom_rotation.setRPY(0.0, 0.0, -0.7);
  odom_base.orientation = tf2::toMsg(odom_rotation);

  const auto map_odom =
    ugv_localization_mvp::mapOdomFromStart(5.0, 4.0, 0.2, 1.2, odom_base);
  tf2::Transform t_map_odom;
  tf2::Transform t_odom_base;
  tf2::fromMsg(map_odom, t_map_odom);
  tf2::fromMsg(odom_base, t_odom_base);
  const tf2::Transform composed = t_map_odom * t_odom_base;

  EXPECT_NEAR(composed.getOrigin().x(), 5.0, kTolerance);
  EXPECT_NEAR(composed.getOrigin().y(), 4.0, kTolerance);
  EXPECT_NEAR(composed.getOrigin().z(), 0.2, kTolerance);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(composed.getRotation()).getRPY(roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.0, kTolerance);
  EXPECT_NEAR(pitch, 0.0, kTolerance);
  EXPECT_NEAR(yaw, 1.2, kTolerance);
}

TEST(TransformMath, NormalizesAcceptedQuaternionScaleBeforeComposition)
{
  geometry_msgs::msg::Pose raw;
  raw.orientation.w = 1.04;
  auto extrinsic =
    ugv_localization_mvp::makeTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.3);
  extrinsic.rotation.x *= 0.98;
  extrinsic.rotation.y *= 0.98;
  extrinsic.rotation.z *= 0.98;
  extrinsic.rotation.w *= 0.98;
  ASSERT_TRUE(ugv_localization_mvp::finiteAndNormalized(raw));
  ASSERT_TRUE(ugv_localization_mvp::finiteAndNormalized(extrinsic));

  const auto result =
    ugv_localization_mvp::odomBaseFromRawLidar(raw, extrinsic);
  const double norm = std::sqrt(
    result.rotation.x * result.rotation.x +
    result.rotation.y * result.rotation.y +
    result.rotation.z * result.rotation.z +
    result.rotation.w * result.rotation.w);
  EXPECT_NEAR(norm, 1.0, kTolerance);
}
}  // namespace
