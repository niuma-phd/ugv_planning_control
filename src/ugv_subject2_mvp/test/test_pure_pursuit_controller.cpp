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

constexpr double kPi = 3.14159265358979323846;

ControlInput make_input(double x = 0.0, double y = 0.0, double yaw = 0.0)
{
  ControlInput input;
  input.inputs_valid = true;
  input.pose.x = x;
  input.pose.y = y;
  input.pose.yaw = yaw;
  return input;
}

TEST(PurePursuitController, DrivesStraightTowardNextOrderedWaypoint)
{
  PurePursuitController controller;
  const auto output = controller.compute(
    make_input(), {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});

  ASSERT_TRUE(output.valid);
  EXPECT_EQ(output.target_index, 1U);
  EXPECT_DOUBLE_EQ(output.target.x, 1.0);
  EXPECT_GT(output.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(output.curvature, 0.0);
  EXPECT_DOUBLE_EQ(output.angular_velocity, 0.0);
}

TEST(PurePursuitController, DefaultCruiseSpeedIsHalfMeterPerSecond)
{
  PurePursuitController controller;
  const auto output = controller.compute(
    make_input(), {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}});

  ASSERT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(controller.config().nominal_speed, 0.5);
  EXPECT_GE(controller.config().max_speed, controller.config().nominal_speed);
  EXPECT_DOUBLE_EQ(output.linear_velocity, 0.5);
}

TEST(PurePursuitController, NeverTargetsPastNextUnconfirmedCsvRow)
{
  PurePursuitController controller;
  const std::vector<Point2D> path{
    {0.0, 0.0}, {0.5, 0.5}, {0.5, -0.5}, {2.0, 0.0}};

  const auto output = controller.compute(make_input(), path);

  ASSERT_TRUE(output.valid);
  EXPECT_EQ(output.target_index, 1U);
  EXPECT_DOUBLE_EQ(output.target.x, path[1].x);
  EXPECT_DOUBLE_EQ(output.target.y, path[1].y);
}

TEST(PurePursuitController, TurnsLeftAndRightWithCorrectSign)
{
  PurePursuitController left_controller;
  const auto left = left_controller.compute(
    make_input(), {{0.0, 0.0}, {1.0, 0.2}, {2.0, 0.8}});
  ASSERT_TRUE(left.valid);
  EXPECT_GT(left.curvature, 0.0);
  EXPECT_GT(left.angular_velocity, 0.0);

  PurePursuitController right_controller;
  const auto right = right_controller.compute(
    make_input(), {{0.0, 0.0}, {1.0, -0.2}, {2.0, -0.8}});
  ASSERT_TRUE(right.valid);
  EXPECT_LT(right.curvature, 0.0);
  EXPECT_LT(right.angular_velocity, 0.0);
}

TEST(PurePursuitController, TurnsInPlaceTowardTargetBehindVehicle)
{
  PurePursuitController left_controller;
  const auto left = left_controller.compute(
    make_input(), {{0.0, 0.0}, {-1.0, 1.0}});
  ASSERT_TRUE(left.valid);
  EXPECT_TRUE(left.turning_in_place);
  EXPECT_DOUBLE_EQ(left.linear_velocity, 0.0);
  EXPECT_GT(left.angular_velocity, 0.0);
  EXPECT_LE(left.angular_velocity, left_controller.config().max_yaw_rate);

  PurePursuitController right_controller;
  const auto right = right_controller.compute(
    make_input(), {{0.0, 0.0}, {-1.0, -1.0}});
  ASSERT_TRUE(right.valid);
  EXPECT_TRUE(right.turning_in_place);
  EXPECT_DOUBLE_EQ(right.linear_velocity, 0.0);
  EXPECT_LT(right.angular_velocity, 0.0);
  EXPECT_GE(right.angular_velocity, -right_controller.config().max_yaw_rate);
}

TEST(PurePursuitController, TurnsInPlaceWhenHeadingErrorExceedsThreshold)
{
  PurePursuitController controller;
  const auto output = controller.compute(
    make_input(), {{0.0, 0.0}, {1.0, 2.0}});

  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.turning_in_place);
  EXPECT_DOUBLE_EQ(output.linear_velocity, 0.0);
  EXPECT_GT(output.angular_velocity, 0.0);
}

TEST(PurePursuitController, ResumesForwardAfterTargetIsAligned)
{
  PurePursuitController controller;
  const std::vector<Point2D> path{{0.0, 0.0}, {-1.0, 1.0}};
  ASSERT_TRUE(controller.compute(make_input(), path).turning_in_place);

  const auto aligned = controller.compute(make_input(0.0, 0.0, 3.0 * kPi / 4.0), path);

  EXPECT_TRUE(aligned.valid);
  EXPECT_FALSE(aligned.turning_in_place);
  EXPECT_GT(aligned.linear_velocity, 0.0);
}

TEST(PurePursuitController, SlowsAndStopsAtEndpoint)
{
  PurePursuitController controller;
  const std::vector<Point2D> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const auto far = controller.compute(make_input(0.0, 0.0), path);
  const auto near = controller.compute(make_input(1.5, 0.0), path);
  const auto reached = controller.compute(make_input(1.95, 0.0), path);

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
  const std::vector<Point2D> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  controller.compute(make_input(0.0, 0.0), path);
  controller.compute(make_input(1.0, 0.0), path);

  const auto output = controller.compute(make_input(2.0, 1.0), path);

  EXPECT_TRUE(output.valid);
  EXPECT_FALSE(output.goal_reached);
  EXPECT_EQ(output.target_index, 2U);
}

TEST(PurePursuitController, StartsAtFirstWaypointWhenInitialPoseIsNearFinalWaypoint)
{
  ControllerConfig config;
  config.waypoint_tolerance = 0.30;
  config.goal_tolerance = 0.30;
  PurePursuitController controller(config);
  const std::vector<Point2D> path{
    {6.5, 0.0}, {6.5, -4.0}, {0.0, -4.0}, {0.0, 0.0}};

  for (int sample = 0; sample < 100; ++sample) {
    const auto output = controller.compute(make_input(0.088, 0.079), path);
    ASSERT_TRUE(output.valid);
    EXPECT_FALSE(output.goal_reached);
    EXPECT_EQ(output.target_index, 0U);
    EXPECT_DOUBLE_EQ(output.target.x, 6.5);
    EXPECT_DOUBLE_EQ(output.target.y, 0.0);
  }
}

TEST(PurePursuitController, ClosedRouteDoesNotFinishAtItsStartingPoint)
{
  ControllerConfig config;
  config.goal_tolerance = 0.30;
  PurePursuitController controller(config);
  const std::vector<Point2D> path{
    {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {0.0, 0.0}};

  for (int sample = 0; sample < 100; ++sample) {
    const auto output = controller.compute(make_input(0.0, 0.0), path);
    ASSERT_TRUE(output.valid);
    EXPECT_FALSE(output.goal_reached);
    EXPECT_EQ(output.target_index, 1U);
    EXPECT_DOUBLE_EQ(output.target.x, 2.0);
    EXPECT_DOUBLE_EQ(output.target.y, 0.0);
  }
}

TEST(PurePursuitController, GoalRequiresOrderedRouteProgress)
{
  PurePursuitController controller;
  const std::vector<Point2D> path{
    {6.5, 0.0}, {6.5, -4.0}, {0.0, -4.0}, {0.0, 0.0}};

  EXPECT_EQ(controller.compute(make_input(0.0, 0.0), path).target_index, 0U);
  EXPECT_EQ(controller.compute(make_input(6.5, 0.0), path).target_index, 1U);

  const auto premature_final = controller.compute(make_input(0.0, 0.0), path);
  EXPECT_FALSE(premature_final.goal_reached);
  EXPECT_EQ(premature_final.target_index, 1U);

  EXPECT_EQ(controller.compute(make_input(6.5, -4.0), path).target_index, 2U);
  EXPECT_EQ(controller.compute(make_input(0.0, -4.0), path).target_index, 3U);

  const auto final = controller.compute(make_input(0.0, 0.0), path);
  EXPECT_TRUE(final.valid);
  EXPECT_TRUE(final.goal_reached);
  EXPECT_DOUBLE_EQ(final.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(final.angular_velocity, 0.0);
}

TEST(PurePursuitController, GoalStaysLatchedUntilProgressReset)
{
  PurePursuitController controller;
  const std::vector<Point2D> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  controller.compute(make_input(0.0, 0.0), path);
  controller.compute(make_input(1.0, 0.0), path);
  ASSERT_TRUE(controller.compute(make_input(2.0, 0.0), path).goal_reached);

  const auto moved_after_goal = controller.compute(make_input(1.0, 0.0), path);
  EXPECT_TRUE(moved_after_goal.goal_reached);
  EXPECT_DOUBLE_EQ(moved_after_goal.linear_velocity, 0.0);

  controller.reset_progress();
  const auto after_reset = controller.compute(make_input(2.0, 0.0), path);
  EXPECT_FALSE(after_reset.goal_reached);
  EXPECT_EQ(after_reset.target_index, 0U);
  EXPECT_DOUBLE_EQ(after_reset.linear_velocity, 0.0);
}

TEST(PurePursuitController, RepeatedIntermediateWaypointsAdvanceInOrder)
{
  PurePursuitController controller;
  const std::vector<Point2D> path{
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  controller.compute(make_input(0.0, 0.0), path);

  const auto repeated = controller.compute(make_input(1.0, 0.0), path);
  EXPECT_TRUE(repeated.valid);
  EXPECT_FALSE(repeated.goal_reached);
  EXPECT_EQ(repeated.target_index, 3U);

  EXPECT_TRUE(controller.compute(make_input(2.0, 0.0), path).goal_reached);
}

TEST(PurePursuitController, IntermediateToleranceIsIndependentFromGoalTolerance)
{
  ControllerConfig config;
  config.waypoint_tolerance = 0.10;
  config.goal_tolerance = 1.00;
  PurePursuitController controller(config);
  const std::vector<Point2D> path{{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}};

  const auto output = controller.compute(make_input(0.5, 0.0), path);

  EXPECT_FALSE(output.goal_reached);
  EXPECT_EQ(output.target_index, 0U);
}

TEST(PurePursuitController, GoalToleranceIsIndependentFromIntermediateTolerance)
{
  ControllerConfig config;
  config.waypoint_tolerance = 1.00;
  config.goal_tolerance = 0.10;
  PurePursuitController controller(config);
  const std::vector<Point2D> path{{0.0, 0.0}, {2.0, 0.0}};
  controller.compute(make_input(0.0, 0.0), path);

  const auto output = controller.compute(make_input(1.5, 0.0), path);

  EXPECT_FALSE(output.goal_reached);
  EXPECT_EQ(output.target_index, 1U);
}

TEST(PurePursuitController, SegmentPassRequiresCrossTrackWithinTolerance)
{
  ControllerConfig config;
  config.waypoint_tolerance = 0.20;
  config.goal_tolerance = 0.10;
  const std::vector<Point2D> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};

  PurePursuitController within(config);
  within.compute(make_input(0.0, 0.0), path);
  EXPECT_EQ(within.compute(make_input(1.1, 0.19), path).target_index, 2U);

  PurePursuitController outside(config);
  outside.compute(make_input(0.0, 0.0), path);
  EXPECT_EQ(outside.compute(make_input(1.1, 0.21), path).target_index, 1U);
}

TEST(PurePursuitController, InvalidInputsAndParametersReturnZero)
{
  PurePursuitController controller;
  auto invalid = make_input();
  invalid.inputs_valid = false;
  const std::vector<Point2D> valid_path{{0.0, 0.0}, {1.0, 0.0}};
  EXPECT_FALSE(controller.compute(invalid, valid_path).valid);

  const std::vector<Point2D> malformed_path{
    {0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0}};
  EXPECT_FALSE(controller.compute(make_input(), malformed_path).valid);

  ControllerConfig zero_waypoint_tolerance;
  zero_waypoint_tolerance.waypoint_tolerance = 0.0;
  EXPECT_FALSE(
    PurePursuitController(zero_waypoint_tolerance).compute(make_input(), valid_path).valid);

  ControllerConfig zero_goal_tolerance;
  zero_goal_tolerance.goal_tolerance = 0.0;
  EXPECT_FALSE(PurePursuitController(zero_goal_tolerance).compute(make_input(), valid_path).valid);

  ControllerConfig unsafe_turn_threshold;
  unsafe_turn_threshold.turn_in_place_threshold_rad = 1.5707963267948966 + 0.01;
  EXPECT_FALSE(
    PurePursuitController(unsafe_turn_threshold).compute(make_input(), valid_path).valid);
}

TEST(PurePursuitController, SharpTurnSlowsToPreserveCurvatureUnderYawRateLimit)
{
  ControllerConfig config;
  config.nominal_speed = 4.0;
  config.max_speed = 0.6;
  config.max_yaw_rate = 0.2;
  config.max_curvature = 0.5;
  config.turn_in_place_threshold_rad = 1.4;
  PurePursuitController controller(config);

  const auto output = controller.compute(
    make_input(), {{0.0, 0.0}, {0.2, 1.0}, {0.4, 2.0}});

  ASSERT_TRUE(output.valid);
  ASSERT_FALSE(output.turning_in_place);
  EXPECT_LE(output.linear_velocity, config.max_speed);
  EXPECT_NEAR(std::abs(output.curvature), config.max_curvature, 1.0e-12);
  EXPECT_LE(std::abs(output.angular_velocity), config.max_yaw_rate);
  EXPECT_NEAR(
    output.angular_velocity / output.linear_velocity, output.curvature, 1.0e-12);
}

}  // namespace
