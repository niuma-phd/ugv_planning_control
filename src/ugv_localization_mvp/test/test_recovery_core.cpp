#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ugv_localization_mvp/recovery_core.hpp"
#include "ugv_localization_mvp/transform_math.hpp"

namespace
{
geometry_msgs::msg::PoseWithCovariance gps(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseWithCovariance result;
  result.pose.position.x = x;
  result.pose.position.y = y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  result.pose.orientation = tf2::toMsg(quaternion);
  result.covariance[0] = 0.25;
  result.covariance[7] = 0.25;
  result.covariance[14] = 0.5;
  result.covariance[35] = 0.04;
  return result;
}
}  // namespace

TEST(RecoveryCore, GpsGateAcceptsBoundedCovariance)
{
  ugv_localization_mvp::RecoveryCoreSettings settings;
  ugv_localization_mvp::RecoveryPose result;
  std::string reason;
  EXPECT_TRUE(ugv_localization_mvp::recoveryPoseFromGps(
    gps(1.0, 2.0, 0.3), 1, settings, result, reason));
  EXPECT_DOUBLE_EQ(result.x, 1.0);
}

TEST(RecoveryCore, InitialInvalidStateRequiresPriorHealthOrStartupGrace)
{
  constexpr double grace_sec = 0.25;
  EXPECT_FALSE(ugv_localization_mvp::initialOdomFaultConfirmed(
    false, false, 0.0, grace_sec));
  EXPECT_FALSE(ugv_localization_mvp::initialOdomFaultConfirmed(
    false, false, 0.24, grace_sec));
  EXPECT_TRUE(ugv_localization_mvp::initialOdomFaultConfirmed(
    false, false, grace_sec, grace_sec));
  EXPECT_TRUE(ugv_localization_mvp::initialOdomFaultConfirmed(
    false, true, 0.0, grace_sec));
  EXPECT_FALSE(ugv_localization_mvp::initialOdomFaultConfirmed(
    true, true, 10.0, grace_sec));
}

TEST(RecoveryCore, GpsGateRejectsUntrustedYaw)
{
  ugv_localization_mvp::RecoveryCoreSettings settings;
  auto sample = gps(1.0, 2.0, 0.3);
  sample.covariance[35] = 0.0;
  ugv_localization_mvp::RecoveryPose result;
  std::string reason;
  EXPECT_FALSE(ugv_localization_mvp::recoveryPoseFromGps(sample, 1, settings, result, reason));
  EXPECT_NE(reason.find("yaw"), std::string::npos);
}

TEST(RecoveryCore, GpsGateRejectsUnpopulatedOrNonFiniteCovariance)
{
  ugv_localization_mvp::RecoveryCoreSettings settings;
  ugv_localization_mvp::RecoveryPose result;
  std::string reason;

  auto zero_position = gps(1.0, 2.0, 0.3);
  zero_position.covariance[0] = 0.0;
  EXPECT_FALSE(ugv_localization_mvp::recoveryPoseFromGps(
    zero_position, 1, settings, result, reason));

  auto nonfinite_cross_term = gps(1.0, 2.0, 0.3);
  nonfinite_cross_term.covariance[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ugv_localization_mvp::recoveryPoseFromGps(
    nonfinite_cross_term, 1, settings, result, reason));

  auto excessive_position = gps(1.0, 2.0, 0.3);
  excessive_position.covariance[7] = settings.max_gps_position_covariance + 0.1;
  EXPECT_FALSE(ugv_localization_mvp::recoveryPoseFromGps(
    excessive_position, 1, settings, result, reason));
}

TEST(RecoveryCore, WindowRequiresStrictStampsAndStability)
{
  ugv_localization_mvp::StablePoseWindow window(3U, 0.5, 0.2);
  EXPECT_FALSE(window.add({1, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_FALSE(window.add({2, 0.1, 0.0, 0.0, 0.05}));
  EXPECT_FALSE(window.add({2, 0.1, 0.0, 0.0, 0.05}));
  EXPECT_TRUE(window.add({3, 0.2, 0.0, 0.0, 0.1}));
  EXPECT_TRUE(window.ready());
  EXPECT_FALSE(window.add({4, 2.0, 0.0, 0.0, 0.1}));
  EXPECT_FALSE(window.ready());
}

TEST(RecoveryCore, WindowChecksYawSpanAcrossEverySamplePair)
{
  ugv_localization_mvp::StablePoseWindow window(3U, 0.5, 0.2);
  EXPECT_FALSE(window.add({1, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_FALSE(window.add({2, 0.0, 0.0, 0.0, 0.19}));
  EXPECT_FALSE(window.add({3, 0.0, 0.0, 0.0, -0.19}));
  EXPECT_FALSE(window.ready());
}

TEST(RecoveryCore, WindowTreatsYawWrapAsAContinuousSmallSpan)
{
  ugv_localization_mvp::StablePoseWindow window(3U, 0.5, 0.05);
  EXPECT_FALSE(window.add({1, 0.0, 0.0, 0.0, M_PI - 0.01}));
  EXPECT_FALSE(window.add({2, 0.0, 0.0, 0.0, -M_PI + 0.01}));
  EXPECT_TRUE(window.add({3, 0.0, 0.0, 0.0, M_PI - 0.02}));
  EXPECT_TRUE(window.ready());
}

TEST(RecoveryCore, ComputesSe2Alignment)
{
  const ugv_localization_mvp::RecoveryPose gps_pose{1, 10.0, 5.0, 2.0, M_PI_2};
  const ugv_localization_mvp::RecoveryPose odom_pose{2, 2.0, 0.0, 0.5, 0.0};
  const auto transform = ugv_localization_mvp::mapOdomFromRecovery(gps_pose, odom_pose);
  EXPECT_NEAR(transform.translation.x, 10.0, 1e-9);
  EXPECT_NEAR(transform.translation.y, 3.0, 1e-9);
  EXPECT_NEAR(transform.translation.z, 1.5, 1e-9);
  EXPECT_NEAR(tf2::getYaw(transform.rotation), M_PI_2, 1e-9);
}

TEST(RecoveryCore, AnchorCorrectionIsBounded)
{
  ugv_localization_mvp::RecoveryCoreSettings settings;
  settings.max_anchor_correction_m = 1.0;
  settings.max_anchor_correction_yaw_rad = 0.2;
  const auto map_odom = ugv_localization_mvp::makeTransform(10.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const ugv_localization_mvp::RecoveryPose trusted{1, 2.0, 0.0, 0.0, 0.0};
  std::string reason;
  EXPECT_TRUE(ugv_localization_mvp::anchorCorrectionAcceptable(
    {2, 12.5, 0.0, 0.0, 0.1}, trusted, map_odom, settings, reason));
  EXPECT_FALSE(ugv_localization_mvp::anchorCorrectionAcceptable(
    {3, 15.0, 0.0, 0.0, 0.1}, trusted, map_odom, settings, reason));
}
