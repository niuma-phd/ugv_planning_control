#include <gtest/gtest.h>

#include "ugv_subject2_mvp/stall_boost_controller.hpp"

namespace ugv_subject2_mvp
{
namespace
{

StallBoostConfig test_config()
{
  StallBoostConfig config;
  config.enabled = true;
  // Most state-transition tests advance time directly. Gap-reset behavior has
  // its own test below with the production-sized threshold.
  config.maximum_observation_gap_sec = 10.0;
  return config;
}

StallBoostInput track_input(
  const double time_sec, const double x = 0.0, const double yaw = 0.0,
  const double linear = 1.5, const double angular = 0.0,
  const std::size_t target_index = 1U)
{
  StallBoostInput input;
  input.steady_time_sec = time_sec;
  input.pose = Pose2D{x, 0.0, yaw};
  input.desired_linear_velocity = linear;
  input.desired_angular_velocity = angular;
  input.target_index = target_index;
  input.command_valid = true;
  return input;
}

StallBoostInput align_input(
  const double time_sec, const double yaw, const double angular)
{
  StallBoostInput input = track_input(time_sec, 0.0, yaw, 0.0, angular);
  input.turning_in_place = true;
  return input;
}

TEST(StallBoostController, RejectsInvalidConfiguration)
{
  auto config = test_config();
  config.boost_max_duration_sec = config.boost_min_duration_sec - 0.01;
  EXPECT_FALSE(StallBoostController(config).config_is_valid());

  config = test_config();
  config.linear_boost_speed_mps = config.minimum_linear_command_mps;
  EXPECT_FALSE(StallBoostController(config).config_is_valid());

  config = test_config();
  config.max_attempts = 0;
  EXPECT_FALSE(StallBoostController(config).config_is_valid());

  config = test_config();
  config.maximum_observation_gap_sec = 0.0;
  EXPECT_FALSE(StallBoostController(config).config_is_valid());
}

TEST(StallBoostController, DisabledControllerPassesCommandsThrough)
{
  StallBoostController controller;
  const auto output = controller.step(track_input(2.0, 0.0, 0.0, 1.5, -1.0));

  EXPECT_DOUBLE_EQ(output.linear_velocity, 1.5);
  EXPECT_DOUBLE_EQ(output.angular_velocity, -1.0);
  EXPECT_EQ(output.phase, StallBoostPhase::idle);
}

TEST(StallBoostController, StationaryTrackBoostsOnlyLinearCommandAfterWindow)
{
  StallBoostController controller(test_config());
  controller.step(track_input(0.0, 0.0, 0.0, 1.5, 1.0));

  const auto before = controller.step(track_input(0.79, 0.0, 0.0, 1.5, 1.0));
  EXPECT_DOUBLE_EQ(before.linear_velocity, 1.5);
  EXPECT_DOUBLE_EQ(before.angular_velocity, 1.0);

  const auto boost = controller.step(track_input(0.81, 0.0, 0.0, 1.5, 1.0));
  EXPECT_EQ(boost.phase, StallBoostPhase::boosting);
  EXPECT_DOUBLE_EQ(boost.linear_velocity, 2.0);
  EXPECT_DOUBLE_EQ(boost.angular_velocity, 1.0);
  EXPECT_TRUE(boost.boost_applied);
}

TEST(StallBoostController, TrackRequiresTranslationRatherThanYawOnlyMotion)
{
  StallBoostController translating(test_config());
  translating.step(track_input(0.0));
  EXPECT_NE(
    translating.step(track_input(0.81, 0.06)).phase,
    StallBoostPhase::boosting);

  StallBoostController yaw_only(test_config());
  yaw_only.step(track_input(0.0));
  const auto boost = yaw_only.step(track_input(0.81, 0.0, 0.04));
  EXPECT_EQ(boost.phase, StallBoostPhase::boosting);
  EXPECT_DOUBLE_EQ(boost.linear_velocity, 2.0);
  EXPECT_DOUBLE_EQ(boost.translation_excursion_m, 0.0);
  EXPECT_NEAR(boost.yaw_excursion_rad, 0.04, 1.0e-12);
}

TEST(StallBoostController, RecoveredMotionRampsMonotonicallyToPlannedCommand)
{
  StallBoostController controller(test_config());
  controller.step(track_input(0.0));
  ASSERT_EQ(controller.step(track_input(0.81)).phase, StallBoostPhase::boosting);
  EXPECT_DOUBLE_EQ(controller.step(track_input(1.20)).linear_velocity, 2.0);

  const auto recovered = controller.step(track_input(1.22, 0.06));
  ASSERT_EQ(recovered.phase, StallBoostPhase::ramp_down);
  EXPECT_DOUBLE_EQ(recovered.linear_velocity, 2.0);

  const auto halfway = controller.step(track_input(1.62, 0.12));
  EXPECT_EQ(halfway.phase, StallBoostPhase::ramp_down);
  EXPECT_NEAR(halfway.linear_velocity, 1.75, 1.0e-12);

  const auto complete = controller.step(track_input(2.03, 0.18));
  EXPECT_EQ(complete.phase, StallBoostPhase::cooldown);
  EXPECT_DOUBLE_EQ(complete.linear_velocity, 1.5);
}

TEST(StallBoostController, LongObservationGapRestartsAFullDetectionWindow)
{
  auto config = test_config();
  config.maximum_observation_gap_sec = 0.30;

  StallBoostController observing(config);
  observing.step(track_input(0.0));
  const auto after_observing_gap = observing.step(track_input(0.50));
  EXPECT_EQ(after_observing_gap.phase, StallBoostPhase::observing);
  EXPECT_DOUBLE_EQ(after_observing_gap.linear_velocity, 1.5);
  observing.step(track_input(0.70));
  observing.step(track_input(0.90));
  observing.step(track_input(1.10));
  EXPECT_EQ(observing.step(track_input(1.31)).phase, StallBoostPhase::boosting);

  StallBoostController boosting(config);
  boosting.step(track_input(0.0));
  boosting.step(track_input(0.20));
  boosting.step(track_input(0.40));
  boosting.step(track_input(0.60));
  ASSERT_EQ(boosting.step(track_input(0.81)).phase, StallBoostPhase::boosting);
  const auto after_boosting_gap = boosting.step(track_input(1.20));
  EXPECT_EQ(after_boosting_gap.phase, StallBoostPhase::observing);
  EXPECT_DOUBLE_EQ(after_boosting_gap.linear_velocity, 1.5);
  EXPECT_FALSE(after_boosting_gap.boost_applied);
}

TEST(StallBoostController, AlignNeverInjectsLinearMotionAndPreservesTurnSign)
{
  for (const double direction_value : {-1.0, 1.0}) {
    StallBoostController controller(test_config());
    controller.step(align_input(0.0, 0.0, direction_value));
    const auto boost = controller.step(align_input(0.81, 0.0, direction_value));

    EXPECT_EQ(boost.mode, StallBoostMode::align);
    EXPECT_EQ(boost.phase, StallBoostPhase::boosting);
    EXPECT_DOUBLE_EQ(boost.linear_velocity, 0.0);
    EXPECT_DOUBLE_EQ(boost.angular_velocity, direction_value * 1.5);
  }
}

TEST(StallBoostController, ZeroInvalidTargetAndDirectionChangesResetBoost)
{
  StallBoostController controller(test_config());
  controller.step(track_input(0.0));
  ASSERT_EQ(controller.step(track_input(0.81)).phase, StallBoostPhase::boosting);

  const auto zero = controller.step(track_input(0.82, 0.0, 0.0, 0.0, 0.0));
  EXPECT_EQ(zero.phase, StallBoostPhase::idle);
  EXPECT_DOUBLE_EQ(zero.linear_velocity, 0.0);

  controller.step(track_input(1.0));
  ASSERT_EQ(controller.step(track_input(1.81)).phase, StallBoostPhase::boosting);
  const auto new_target =
    controller.step(track_input(1.82, 0.0, 0.0, 1.5, 0.0, 2U));
  EXPECT_EQ(new_target.phase, StallBoostPhase::observing);
  EXPECT_DOUBLE_EQ(new_target.linear_velocity, 1.5);

  const auto reversed = controller.step(track_input(1.83, 0.0, 0.0, -1.5));
  EXPECT_EQ(reversed.phase, StallBoostPhase::observing);
  EXPECT_DOUBLE_EQ(reversed.linear_velocity, -1.5);

  auto invalid = track_input(1.84);
  invalid.command_valid = false;
  const auto stopped = controller.step(invalid);
  EXPECT_EQ(stopped.phase, StallBoostPhase::idle);
  EXPECT_DOUBLE_EQ(stopped.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(stopped.angular_velocity, 0.0);
}

TEST(StallBoostController, FailedAttemptsAreBoundedAndThenSuppressed)
{
  auto config = test_config();
  config.max_attempts = 1;
  StallBoostController controller(config);
  controller.step(track_input(0.0));
  ASSERT_EQ(controller.step(track_input(0.81)).phase, StallBoostPhase::boosting);
  EXPECT_EQ(controller.step(track_input(1.82)).phase, StallBoostPhase::ramp_down);
  EXPECT_EQ(controller.step(track_input(2.63)).phase, StallBoostPhase::cooldown);
  EXPECT_EQ(controller.step(track_input(3.64)).phase, StallBoostPhase::observing);

  const auto suppressed = controller.step(track_input(4.45));
  EXPECT_EQ(suppressed.phase, StallBoostPhase::suppressed);
  EXPECT_DOUBLE_EQ(suppressed.linear_velocity, 1.5);
  EXPECT_FALSE(suppressed.boost_applied);
}

}  // namespace
}  // namespace ugv_subject2_mvp
