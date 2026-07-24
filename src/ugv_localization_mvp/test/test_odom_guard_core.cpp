#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "ugv_localization_mvp/odom_guard_core.hpp"

namespace
{
using ugv_localization_mvp::OdomFault;
using ugv_localization_mvp::OdomGuardCore;
using ugv_localization_mvp::OdomGuardSettings;
using ugv_localization_mvp::OdomSample;
constexpr std::int64_t kSecond = 1000000000LL;

OdomSample sampleAt(std::int64_t stamp, double x = 0.0, double yaw = 0.0)
{
  OdomSample sample;
  sample.stamp_ns = stamp;
  sample.x = x;
  sample.qz = std::sin(yaw / 2.0);
  sample.qw = std::cos(yaw / 2.0);
  return sample;
}

TEST(OdomGuardCore, AcceptsOrderedFiniteSamples)
{
  OdomGuardCore guard;
  EXPECT_EQ(guard.evaluate(sampleAt(kSecond), kSecond), OdomFault::kNone);
  EXPECT_EQ(guard.evaluate(sampleAt(2 * kSecond, 0.2, 0.1), 2 * kSecond), OdomFault::kNone);
  EXPECT_FALSE(guard.latched());
}

TEST(OdomGuardCore, LatchesAndRefusesFurtherSamples)
{
  OdomGuardCore guard;
  EXPECT_EQ(guard.evaluate(sampleAt(kSecond), kSecond), OdomFault::kNone);
  auto invalid = sampleAt(2 * kSecond);
  invalid.x = std::nan("");
  EXPECT_EQ(guard.evaluate(invalid, 2 * kSecond), OdomFault::kNonFinite);
  EXPECT_TRUE(guard.latched());
  EXPECT_EQ(guard.evaluate(sampleAt(3 * kSecond), 3 * kSecond), OdomFault::kAlreadyLatched);
}

TEST(OdomGuardCore, RejectsStamps)
{
  OdomGuardCore stale;
  EXPECT_EQ(stale.evaluate(sampleAt(kSecond), 2 * kSecond), OdomFault::kStale);
  OdomGuardCore future;
  EXPECT_EQ(future.evaluate(sampleAt(2 * kSecond), kSecond), OdomFault::kFutureStamp);
  OdomGuardCore repeated;
  EXPECT_EQ(repeated.evaluate(sampleAt(kSecond), kSecond), OdomFault::kNone);
  EXPECT_EQ(repeated.evaluate(sampleAt(kSecond), kSecond), OdomFault::kRepeatedStamp);
  OdomGuardCore backward;
  EXPECT_EQ(backward.evaluate(sampleAt(2 * kSecond), 2 * kSecond), OdomFault::kNone);
  EXPECT_EQ(backward.evaluate(sampleAt(1900000000LL), 2 * kSecond), OdomFault::kBackwardStamp);
}

TEST(OdomGuardCore, RejectsPoseJumpsAndBadQuaternion)
{
  OdomGuardSettings settings;
  settings.max_translation_jump_m = 0.5;
  settings.max_yaw_jump_rad = 0.3;
  OdomGuardCore translation(settings);
  EXPECT_EQ(translation.evaluate(sampleAt(kSecond), kSecond), OdomFault::kNone);
  EXPECT_EQ(translation.evaluate(sampleAt(2 * kSecond, 0.6), 2 * kSecond), OdomFault::kTranslationJump);
  OdomGuardCore rotation(settings);
  EXPECT_EQ(rotation.evaluate(sampleAt(kSecond), kSecond), OdomFault::kNone);
  EXPECT_EQ(rotation.evaluate(sampleAt(2 * kSecond, 0.0, 0.4), 2 * kSecond), OdomFault::kYawJump);
  OdomGuardCore quaternion(settings);
  auto bad = sampleAt(kSecond);
  bad.qw = 2.0;
  EXPECT_EQ(quaternion.evaluate(bad, kSecond), OdomFault::kInvalidQuaternion);
}

TEST(OdomGuardCore, ResetRequiresFreshBaseline)
{
  OdomGuardCore guard;
  auto invalid = sampleAt(kSecond);
  invalid.auxiliary_finite = false;
  EXPECT_EQ(guard.evaluate(invalid, kSecond), OdomFault::kNonFinite);
  guard.reset();
  EXPECT_FALSE(guard.latched());
  EXPECT_EQ(guard.evaluate(sampleAt(10 * kSecond, 100.0), 10 * kSecond), OdomFault::kNone);
}
}  // namespace
