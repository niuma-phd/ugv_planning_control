#include "ugv_subject1_avoidance_mvp/local_avoidance.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

using ugv_subject1_avoidance_mvp::LocalAvoidance;
using ugv_subject1_avoidance_mvp::NominalCommand;
using ugv_subject1_avoidance_mvp::PlanResult;
using ugv_subject1_avoidance_mvp::PlannerConfig;
using ugv_subject1_avoidance_mvp::Point2;
using ugv_subject1_avoidance_mvp::frame_id_matches;
using ugv_subject1_avoidance_mvp::select_final_command;

namespace
{
PlannerConfig test_config()
{
  PlannerConfig config;
  config.speed_mps = 0.4;
  config.max_curvature = 1.0;
  config.curvature_samples = 21;
  config.horizon_m = 3.0;
  config.step_m = 0.05;
  config.footprint_half_length_m = 0.30;
  config.footprint_half_width_m = 0.25;
  config.inflation_m = 0.10;
  config.goal_distance_weight = 1.2;
  config.heading_weight = 0.8;
  config.curvature_weight = 0.1;
  config.clearance_weight = 0.4;
  return config;
}
}  // namespace

TEST(LocalAvoidance, EmptyFreshObstacleInputDoesNotRequestControl)
{
  LocalAvoidance planner(test_config());
  const auto result = planner.plan({}, {4.0, 0.0}, true);
  EXPECT_FALSE(result.active);
  EXPECT_FALSE(result.has_safe_trajectory);
  EXPECT_DOUBLE_EQ(result.speed_mps, 0.0);
  EXPECT_TRUE(result.trajectory.empty());
}

TEST(LocalAvoidance, StaleInputRequestsStop)
{
  LocalAvoidance planner(test_config());
  const auto result = planner.plan({}, {4.0, 0.0}, false);
  EXPECT_TRUE(result.active);
  EXPECT_FALSE(result.has_safe_trajectory);
  EXPECT_DOUBLE_EQ(result.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.yaw_rate_radps, 0.0);
}

TEST(LocalAvoidance, FrameIdMustMatchExactly)
{
  EXPECT_TRUE(frame_id_matches("base_link", "base_link"));
  EXPECT_FALSE(frame_id_matches("", "base_link"));
  EXPECT_FALSE(frame_id_matches("/base_link", "base_link"));
  EXPECT_FALSE(frame_id_matches("odom", "base_link"));
  EXPECT_FALSE(frame_id_matches("base_link", ""));
}

TEST(LocalAvoidance, NonFiniteGoalRequestsStop)
{
  LocalAvoidance planner(test_config());
  const auto result = planner.plan(
    {{1.0, 0.0}}, {std::numeric_limits<double>::quiet_NaN(), 0.0}, true);
  EXPECT_TRUE(result.active);
  EXPECT_FALSE(result.has_safe_trajectory);
  EXPECT_DOUBLE_EQ(result.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.yaw_rate_radps, 0.0);
}

TEST(LocalAvoidance, NonFiniteObstacleRequestsStop)
{
  LocalAvoidance planner(test_config());
  const auto result = planner.plan(
    {{1.0, std::numeric_limits<double>::infinity()}}, {4.0, 0.0}, true);
  EXPECT_TRUE(result.active);
  EXPECT_FALSE(result.has_safe_trajectory);
  EXPECT_DOUBLE_EQ(result.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.yaw_rate_radps, 0.0);
}

TEST(LocalAvoidance, NonFiniteConfigurationIsRejected)
{
  auto config = test_config();
  config.clearance_weight = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(LocalAvoidance planner(config), std::invalid_argument);
}

TEST(LocalAvoidance, RejectsCollisionSamplingWithLongitudinalGaps)
{
  auto config = test_config();
  config.step_m =
    2.0 * (config.footprint_half_length_m + config.inflation_m) + 0.01;
  EXPECT_THROW(LocalAvoidance planner(config), std::invalid_argument);
}

TEST(LocalAvoidance, RejectsUnboundedConfiguredRolloutWork)
{
  auto huge_horizon = test_config();
  huge_horizon.horizon_m = std::numeric_limits<double>::max();
  EXPECT_THROW(LocalAvoidance planner(huge_horizon), std::invalid_argument);

  auto too_many_curvatures = test_config();
  too_many_curvatures.curvature_samples = 1002;
  EXPECT_THROW(LocalAvoidance planner(too_many_curvatures), std::invalid_argument);
}

TEST(LocalAvoidance, DoesNotTakeControlForObstacleOutsideNominalSweep)
{
  LocalAvoidance planner(test_config());
  const auto result = planner.plan({{1.5, 1.4}}, {4.0, 0.0}, true);
  EXPECT_FALSE(result.active);
  EXPECT_FALSE(result.has_safe_trajectory);
  EXPECT_DOUBLE_EQ(result.speed_mps, 0.0);
}

TEST(LocalAvoidance, NominalSweepDefinesRelevanceWithoutASecondCorridor)
{
  auto config = test_config();
  config.footprint_half_length_m = 1.0;
  config.footprint_half_width_m = 0.6;
  config.inflation_m = 0.3;
  LocalAvoidance planner(config);
  const auto result = planner.plan({{4.2, 0.85}}, {5.0, 0.0}, true);
  EXPECT_TRUE(result.active);
}

TEST(LocalAvoidance, GoesLeftAroundObstacleOnRight)
{
  LocalAvoidance planner(test_config());
  const auto result = planner.plan({{1.2, -0.12}}, {4.0, 0.6}, true);
  ASSERT_TRUE(result.has_safe_trajectory);
  EXPECT_GT(result.curvature, 0.0);
  EXPECT_GT(result.yaw_rate_radps, 0.0);
}

TEST(LocalAvoidance, GoesRightAroundObstacleOnLeft)
{
  LocalAvoidance planner(test_config());
  const auto result = planner.plan({{1.2, 0.12}}, {4.0, -0.6}, true);
  ASSERT_TRUE(result.has_safe_trajectory);
  EXPECT_LT(result.curvature, 0.0);
  EXPECT_LT(result.yaw_rate_radps, 0.0);
}

TEST(LocalAvoidance, FullyBlockedReturnsActiveZeroCommand)
{
  auto config = test_config();
  config.max_curvature = 0.8;
  LocalAvoidance planner(config);
  std::vector<Point2> wall;
  for (double x = 0.35; x <= 3.0; x += 0.25) {
    for (double y = -2.0; y <= 2.0; y += 0.20) {
      wall.push_back({x, y});
    }
  }
  const auto result = planner.plan(wall, {4.0, 0.0}, true);
  EXPECT_TRUE(result.active);
  EXPECT_FALSE(result.has_safe_trajectory);
  EXPECT_DOUBLE_EQ(result.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.yaw_rate_radps, 0.0);
  EXPECT_TRUE(result.trajectory.empty());
}

TEST(CommandSelection, PassesThroughFreshPlanarNominalWhenAvoidanceIsInactive)
{
  const auto command = select_final_command(
    PlanResult{}, NominalCommand{0.7, 0.0, 0.0, 0.0, 0.0, -0.3}, true);
  EXPECT_DOUBLE_EQ(command.linear_x_mps, 0.7);
  EXPECT_DOUBLE_EQ(command.angular_z_radps, -0.3);
}

TEST(CommandSelection, ActiveAvoidanceAtomicallySelectsLocalCandidate)
{
  PlanResult plan;
  plan.active = true;
  plan.has_safe_trajectory = true;
  plan.speed_mps = 0.2;
  plan.yaw_rate_radps = 0.4;
  const auto command = select_final_command(
    plan, NominalCommand{0.8, 0.0, 0.0, 0.0, 0.0, -0.2}, true);
  EXPECT_DOUBLE_EQ(command.linear_x_mps, 0.2);
  EXPECT_DOUBLE_EQ(command.angular_z_radps, 0.4);
}

TEST(CommandSelection, ActiveBlockedOrInvalidCandidateNeverFallsBackToNominal)
{
  PlanResult blocked;
  blocked.active = true;
  const auto blocked_command = select_final_command(
    blocked, NominalCommand{0.8, 0.0, 0.0, 0.0, 0.0, -0.2}, true);
  EXPECT_DOUBLE_EQ(blocked_command.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(blocked_command.angular_z_radps, 0.0);

  blocked.has_safe_trajectory = true;
  blocked.speed_mps = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_candidate = select_final_command(
    blocked, NominalCommand{0.8, 0.0, 0.0, 0.0, 0.0, -0.2}, true);
  EXPECT_DOUBLE_EQ(invalid_candidate.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(invalid_candidate.angular_z_radps, 0.0);
}

TEST(CommandSelection, StaleOrInvalidNominalStopsWhenAvoidanceIsInactive)
{
  const NominalCommand valid{0.7, 0.0, 0.0, 0.0, 0.0, -0.3};
  const auto stale = select_final_command(PlanResult{}, valid, false);
  EXPECT_DOUBLE_EQ(stale.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(stale.angular_z_radps, 0.0);

  auto non_finite = valid;
  non_finite.angular_z = std::numeric_limits<double>::infinity();
  const auto invalid = select_final_command(PlanResult{}, non_finite, true);
  EXPECT_DOUBLE_EQ(invalid.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(invalid.angular_z_radps, 0.0);

  auto non_planar = valid;
  non_planar.linear_y = 0.1;
  const auto invalid_plane = select_final_command(PlanResult{}, non_planar, true);
  EXPECT_DOUBLE_EQ(invalid_plane.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(invalid_plane.angular_z_radps, 0.0);
}
