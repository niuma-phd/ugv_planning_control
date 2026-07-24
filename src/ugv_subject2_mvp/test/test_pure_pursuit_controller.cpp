#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "ugv_subject2_mvp/pure_pursuit_controller.hpp"

namespace
{
using ugv_subject2_mvp::ControlInput;
using ugv_subject2_mvp::ControllerConfig;
using ugv_subject2_mvp::Point2D;
using ugv_subject2_mvp::PurePursuitController;

ControlInput make_input(std::vector<Point2D> path, double x = 0.0, double y = 0.0)
{
  ControlInput input;
  input.inputs_valid = true;
  input.pose.x = x;
  input.pose.y = y;
  input.pose.yaw = 0.0;
  input.path = std::move(path);
  return input;
}

TEST(PurePursuitController, DrivesStraightForStraightPath)
{
  PurePursuitController controller;
  const auto output = controller.compute(make_input({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}));
  ASSERT_TRUE(output.valid);
  EXPECT_GT(output.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(output.curvature, 0.0);
  EXPECT_DOUBLE_EQ(output.angular_velocity, 0.0);
}

TEST(PurePursuitController, TurnsLeftAndRightWithCorrectSign)
{
  PurePursuitController left_controller;
  const auto left = left_controller.compute(
    make_input({{0.0, 0.0}, {0.7, 0.2}, {1.4, 0.8}}));
  ASSERT_TRUE(left.valid);
  EXPECT_GT(left.curvature, 0.0);
  EXPECT_GT(left.angular_velocity, 0.0);

  PurePursuitController right_controller;
  const auto right = right_controller.compute(
    make_input({{0.0, 0.0}, {0.7, -0.2}, {1.4, -0.8}}));
  ASSERT_TRUE(right.valid);
  EXPECT_LT(right.curvature, 0.0);
  EXPECT_LT(right.angular_velocity, 0.0);
}

TEST(PurePursuitController, StopsWhenLookaheadTargetIsBehindVehicle)
{
  PurePursuitController controller;
  auto input = make_input({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  input.pose.yaw = 3.14159265358979323846;

  const auto output = controller.compute(input);

  EXPECT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(output.angular_velocity, 0.0);
}

TEST(PurePursuitController, SlowsAndStopsAtEndpoint)
{
  PurePursuitController controller;
  const std::vector<Point2D> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const auto far = controller.compute(make_input(path, 0.0, 0.0));
  controller.reset_progress();
  const auto near = controller.compute(make_input(path, 1.5, 0.0));
  controller.reset_progress();
  const auto reached = controller.compute(make_input(path, 1.95, 0.0));

  EXPECT_GT(far.linear_velocity, near.linear_velocity);
  EXPECT_GT(near.linear_velocity, 0.0);
  EXPECT_TRUE(reached.valid);
  EXPECT_TRUE(reached.goal_reached);
  EXPECT_DOUBLE_EQ(reached.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(reached.angular_velocity, 0.0);
}

TEST(PurePursuitController, DoesNotDeclareGoalWhenLaterallyFarFromEndpoint)
{
  PurePursuitController controller;
  const auto output = controller.compute(
    make_input({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}, 2.0, 1.0));
  EXPECT_TRUE(output.valid);
  EXPECT_FALSE(output.goal_reached);
  EXPECT_GT(output.linear_velocity, 0.0);
}

TEST(PurePursuitController, InvalidAndMalformedInputsReturnZero)
{
  PurePursuitController controller;
  auto invalid = make_input({{0.0, 0.0}, {1.0, 0.0}});
  invalid.inputs_valid = false;
  const auto invalid_output = controller.compute(invalid);
  EXPECT_FALSE(invalid_output.valid);
  EXPECT_DOUBLE_EQ(invalid_output.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(invalid_output.angular_velocity, 0.0);

  auto malformed = make_input({{0.0, 0.0}, {1.0, 0.0}});
  malformed.path[1].x = std::numeric_limits<double>::quiet_NaN();
  const auto malformed_output = controller.compute(malformed);
  EXPECT_FALSE(malformed_output.valid);
  EXPECT_DOUBLE_EQ(malformed_output.linear_velocity, 0.0);
}

TEST(PurePursuitController, EnforcesSpeedYawRateAndCurvatureLimits)
{
  ControllerConfig config;
  config.nominal_speed = 4.0;
  config.max_speed = 0.6;
  config.max_yaw_rate = 0.2;
  config.max_curvature = 0.5;
  PurePursuitController controller(config);

  const auto output = controller.compute(
    make_input({{0.0, 0.0}, {0.2, 1.0}, {0.4, 2.0}}));
  ASSERT_TRUE(output.valid);
  EXPECT_LE(output.linear_velocity, config.max_speed);
  EXPECT_LE(std::abs(output.curvature), config.max_curvature);
  EXPECT_LE(std::abs(output.angular_velocity), config.max_yaw_rate);
}

TEST(PurePursuitController, SpeedScaledLookaheadSelectsFartherTarget)
{
  ControllerConfig config;
  config.lookahead_distance = 0.5;
  config.lookahead_speed_gain = 1.0;
  config.min_lookahead = 0.5;
  config.max_lookahead = 3.0;
  PurePursuitController controller(config);
  auto input = make_input({{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}, {2.0, 0.0}});
  input.current_speed = 1.0;
  const auto output = controller.compute(input);
  EXPECT_GE(output.target_index, 3U);
}

}  // namespace
