#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "ugv_localization_mvp/odom_guard_core.hpp"

namespace
{
using ugv_localization_mvp::OdomFault;
using ugv_localization_mvp::OdomGuardCore;
using ugv_localization_mvp::OdomGuardSettings;
using ugv_localization_mvp::OdomSample;
OdomSample sampleAt(double x = 0.0, double yaw = 0.0)
{
  OdomSample sample;
  sample.x = x;
  sample.qz = std::sin(yaw / 2.0);
  sample.qw = std::cos(yaw / 2.0);
  return sample;
}

TEST(OdomGuardCore, AcceptsOrderedFiniteSamples)
{
  OdomGuardCore guard;
  EXPECT_EQ(guard.evaluate(sampleAt()), OdomFault::kNone);
  EXPECT_EQ(guard.evaluate(sampleAt(0.2, 0.1)), OdomFault::kNone);
  EXPECT_FALSE(guard.latched());
}

TEST(OdomGuardCore, LatchesAndRefusesFurtherSamples)
{
  OdomGuardCore guard;
  EXPECT_EQ(guard.evaluate(sampleAt()), OdomFault::kNone);
  auto invalid = sampleAt();
  invalid.x = std::nan("");
  EXPECT_EQ(guard.evaluate(invalid), OdomFault::kNonFinite);
  EXPECT_TRUE(guard.latched());
  EXPECT_EQ(guard.evaluate(sampleAt()), OdomFault::kAlreadyLatched);
}

TEST(OdomGuardCore, RejectsPoseJumpsAndBadQuaternion)
{
  OdomGuardSettings settings;
  settings.max_translation_jump_m = 0.5;
  settings.max_yaw_jump_rad = 0.3;
  OdomGuardCore translation(settings);
  EXPECT_EQ(translation.evaluate(sampleAt()), OdomFault::kNone);
  EXPECT_EQ(translation.evaluate(sampleAt(0.6)), OdomFault::kTranslationJump);
  OdomGuardCore rotation(settings);
  EXPECT_EQ(rotation.evaluate(sampleAt()), OdomFault::kNone);
  EXPECT_EQ(rotation.evaluate(sampleAt(0.0, 0.4)), OdomFault::kYawJump);
  OdomGuardCore quaternion(settings);
  auto bad = sampleAt();
  bad.qw = 2.0;
  EXPECT_EQ(quaternion.evaluate(bad), OdomFault::kInvalidQuaternion);
}

TEST(OdomGuardCore, NormalizesAcceptedQuaternionScaleForYawComparison)
{
  OdomGuardSettings settings;
  settings.max_yaw_jump_rad = 0.15;
  OdomGuardCore guard(settings);
  auto first = sampleAt(0.0, 0.40);
  auto second = sampleAt(0.0, 0.50);
  for (auto * sample : {&first, &second}) {
    sample->qz *= 1.04;
    sample->qw *= 1.04;
  }
  EXPECT_EQ(guard.evaluate(first), OdomFault::kNone);
  EXPECT_EQ(guard.evaluate(second), OdomFault::kNone);
}

TEST(OdomGuardCore, ResetRequiresNewBaseline)
{
  OdomGuardCore guard;
  auto invalid = sampleAt();
  invalid.auxiliary_finite = false;
  EXPECT_EQ(guard.evaluate(invalid), OdomFault::kNonFinite);
  guard.reset();
  EXPECT_FALSE(guard.latched());
  EXPECT_EQ(guard.evaluate(sampleAt(100.0)), OdomFault::kNone);
}

TEST(OdomGuardCore, RejectsNonFiniteQuaternionNormTolerance)
{
  OdomGuardSettings settings;
  settings.quaternion_norm_tolerance = -std::numeric_limits<double>::infinity();
  EXPECT_THROW(OdomGuardCore guard(settings), std::invalid_argument);
}

TEST(OdomGuardCore, RejectsNonFiniteTranslationJumpThreshold)
{
  OdomGuardSettings settings;
  settings.max_translation_jump_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(OdomGuardCore guard(settings), std::invalid_argument);
}

TEST(OdomGuardCore, RejectsNonFiniteYawJumpThreshold)
{
  OdomGuardSettings settings;
  settings.max_yaw_jump_rad = std::numeric_limits<double>::infinity();
  EXPECT_THROW(OdomGuardCore guard(settings), std::invalid_argument);
}
}  // namespace
