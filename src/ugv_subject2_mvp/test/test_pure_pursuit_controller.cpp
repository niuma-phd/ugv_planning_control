#include <algorithm>
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
using ugv_subject2_mvp::TrackingYawPulseShaper;

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

ControllerConfig enhanced_config()
{
  ControllerConfig config;
  config.enhanced_tracking_enabled = true;
  config.max_yaw_rate = 1.5;
  config.turn_in_place_threshold_rad = 0.70;
  config.turn_in_place_exit_threshold_rad = 0.20;
  config.lookahead_min_m = 1.0;
  config.lookahead_max_m = 1.0;
  config.lookahead_speed_gain = 0.0;
  config.tracking_omega_enter_threshold_rad_s = 0.05;
  config.tracking_omega_exit_threshold_rad_s = 0.02;
  return config;
}

TEST(TrackingYawPulseShaper, RejectsInvalidConfiguration)
{
  TrackingYawPulseShaper zero_rate(0.0, 1.0, 0.10);
  TrackingYawPulseShaper zero_amplitude(20.0, 0.0, 0.10);
  TrackingYawPulseShaper zero_duration(20.0, 1.0, 0.0);

  EXPECT_FALSE(zero_rate.config_is_valid());
  EXPECT_FALSE(zero_amplitude.config_is_valid());
  EXPECT_FALSE(zero_duration.config_is_valid());
  EXPECT_DOUBLE_EQ(zero_rate.step(0.2, true), 0.0);
}

TEST(TrackingYawPulseShaper, PassesDemandAtOrAbovePhysicalMinimum)
{
  TrackingYawPulseShaper shaper(20.0, 1.0, 0.10);

  EXPECT_DOUBLE_EQ(shaper.step(1.0, true), 1.0);
  EXPECT_DOUBLE_EQ(shaper.step(-1.2, true), -1.2);
}

TEST(TrackingYawPulseShaper, ConvertsSubFloorDemandIntoMinimumLengthPulses)
{
  TrackingYawPulseShaper shaper(20.0, 1.0, 0.10);
  constexpr int sample_count = 200;
  int nonzero_samples = 0;
  int shortest_pulse_run = sample_count;
  int current_pulse_run = 0;
  double sum = 0.0;

  for (int sample = 0; sample < sample_count; ++sample) {
    const double command = shaper.step(0.2, true);
    EXPECT_TRUE(command == 0.0 || command == 1.0);
    sum += command;
    if (command != 0.0) {
      ++nonzero_samples;
      ++current_pulse_run;
    } else if (current_pulse_run > 0) {
      shortest_pulse_run = std::min(shortest_pulse_run, current_pulse_run);
      current_pulse_run = 0;
    }
  }
  // The finite observation window may end in the middle of a two-tick pulse,
  // so only completed runs participate in the minimum-duration assertion.
  EXPECT_GE(nonzero_samples, 39);
  EXPECT_LE(nonzero_samples, 40);
  EXPECT_EQ(shortest_pulse_run, 2);
  EXPECT_LE(
    std::abs(sum / static_cast<double>(sample_count) - 0.2),
    1.0 / static_cast<double>(sample_count) + 1.0e-12);
}

TEST(TrackingYawPulseShaper, InactiveCorrectionResetsPendingPulse)
{
  TrackingYawPulseShaper shaper(20.0, 1.0, 0.10);
  for (int sample = 0; sample < 9; ++sample) {
    EXPECT_DOUBLE_EQ(shaper.step(0.2, true), 0.0);
  }

  EXPECT_DOUBLE_EQ(shaper.step(0.2, false), 0.0);
  for (int sample = 0; sample < 9; ++sample) {
    EXPECT_DOUBLE_EQ(shaper.step(0.2, true), 0.0);
  }
  EXPECT_DOUBLE_EQ(shaper.step(0.2, true), 1.0);
}

TEST(TrackingYawPulseShaper, DirectionChangeDropsResidualDemand)
{
  TrackingYawPulseShaper shaper(20.0, 1.0, 0.10);
  for (int sample = 0; sample < 9; ++sample) {
    EXPECT_DOUBLE_EQ(shaper.step(0.2, true), 0.0);
  }

  for (int sample = 0; sample < 9; ++sample) {
    EXPECT_DOUBLE_EQ(shaper.step(-0.2, true), 0.0);
  }
  EXPECT_DOUBLE_EQ(shaper.step(-0.2, true), -1.0);
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

TEST(PurePursuitController, StandardPursuitRejectsInvalidLookaheadAndDeadbandParameters)
{
  ControllerConfig config = enhanced_config();
  config.max_yaw_rate = 1.49;
  EXPECT_FALSE(PurePursuitController(config).config_is_valid());

  config = enhanced_config();
  config.lookahead_max_m = config.lookahead_min_m - 0.01;
  EXPECT_FALSE(PurePursuitController(config).config_is_valid());

  config = enhanced_config();
  config.lookahead_speed_gain = -0.01;
  EXPECT_FALSE(PurePursuitController(config).config_is_valid());

  config = enhanced_config();
  config.tracking_omega_exit_threshold_rad_s =
    config.tracking_omega_enter_threshold_rad_s;
  EXPECT_FALSE(PurePursuitController(config).config_is_valid());
}

TEST(PurePursuitController, StandardPursuitUsesPlannedSpeedForBoundedLookahead)
{
  ControllerConfig config = enhanced_config();
  config.lookahead_min_m = 1.0;
  config.lookahead_max_m = 3.0;
  config.lookahead_speed_gain = 2.0;
  PurePursuitController controller(config);

  const auto output = controller.compute(
    make_input(), {{0.0, 0.0}, {10.0, 0.0}});

  ASSERT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.lookahead_distance, 2.0);
  EXPECT_DOUBLE_EQ(output.pursuit_target.x, 2.0);
  EXPECT_DOUBLE_EQ(output.pursuit_target.y, 0.0);
}

TEST(PurePursuitController, StandardPursuitProjectsAndInterpolatesSparseStraightPath)
{
  PurePursuitController controller(enhanced_config());
  const std::vector<Point2D> path{{0.0, 0.0}, {100.0, 0.0}};
  controller.compute(make_input(), path);

  const auto output = controller.compute(make_input(10.0, 2.0), path);

  ASSERT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.path_projection.x, 10.0);
  EXPECT_DOUBLE_EQ(output.path_projection.y, 0.0);
  EXPECT_DOUBLE_EQ(output.pursuit_target.x, 11.0);
  EXPECT_DOUBLE_EQ(output.pursuit_target.y, 0.0);
  EXPECT_DOUBLE_EQ(output.cross_track_error, 2.0);
  EXPECT_NEAR(output.curvature, -0.8, 1.0e-12);
  EXPECT_DOUBLE_EQ(output.angular_velocity, -1.0);
}

TEST(PurePursuitController, StandardPursuitIsInsensitiveToCollinearPathDensity)
{
  const std::vector<Point2D> sparse{{0.0, 0.0}, {100.0, 0.0}};
  std::vector<Point2D> dense;
  for (int x = 0; x <= 100; ++x) {
    dense.push_back(Point2D{static_cast<double>(x), 0.0});
  }

  PurePursuitController sparse_controller(enhanced_config());
  sparse_controller.compute(make_input(), sparse);
  const auto sparse_output = sparse_controller.compute(make_input(10.0, 0.2), sparse);

  PurePursuitController dense_controller(enhanced_config());
  dense_controller.compute(make_input(), dense);
  const auto dense_output = dense_controller.compute(make_input(10.0, 0.2), dense);

  EXPECT_NEAR(sparse_output.path_projection.x, dense_output.path_projection.x, 1.0e-12);
  EXPECT_NEAR(sparse_output.pursuit_target.x, dense_output.pursuit_target.x, 1.0e-12);
  EXPECT_NEAR(sparse_output.pursuit_target.y, dense_output.pursuit_target.y, 1.0e-12);
  EXPECT_NEAR(sparse_output.curvature, dense_output.curvature, 1.0e-12);
}

TEST(PurePursuitController, StandardPursuitStopsPreviewAtUnconfirmedCheckpoint)
{
  ControllerConfig config = enhanced_config();
  config.lookahead_min_m = 2.0;
  config.lookahead_max_m = 2.0;
  PurePursuitController controller(config);
  const std::vector<Point2D> path{{0.0, 0.0}, {2.0, 0.0}, {2.0, 3.0}};
  controller.compute(make_input(), path);

  const auto output = controller.compute(make_input(1.0, 0.0), path);

  ASSERT_TRUE(output.valid);
  EXPECT_EQ(output.target_index, 1U);
  EXPECT_EQ(output.pursuit_segment_index, 1U);
  EXPECT_DOUBLE_EQ(output.path_projection.x, 1.0);
  EXPECT_DOUBLE_EQ(output.pursuit_target.x, 2.0);
  EXPECT_DOUBLE_EQ(output.pursuit_target.y, 0.0);
  EXPECT_DOUBLE_EQ(output.curvature, 0.0);
}

TEST(PurePursuitController, StandardPursuitDoesNotDeadlockOnUnconfirmedHairpin)
{
  ControllerConfig config = enhanced_config();
  config.lookahead_min_m = 2.0;
  config.lookahead_max_m = 2.0;
  PurePursuitController controller(config);
  const std::vector<Point2D> path{{0.0, 0.0}, {1.0, 0.0}, {-2.0, 0.0}};

  const auto output = controller.compute(make_input(), path);

  ASSERT_TRUE(output.valid);
  EXPECT_EQ(output.target_index, 1U);
  EXPECT_DOUBLE_EQ(output.pursuit_target.x, 1.0);
  EXPECT_DOUBLE_EQ(output.pursuit_target.y, 0.0);
  EXPECT_GT(output.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(output.angular_velocity, 0.0);
}

TEST(PurePursuitController, StandardPursuitReturnsToMissedOrderedCheckpoint)
{
  PurePursuitController controller(enhanced_config());
  const std::vector<Point2D> path{{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}};
  controller.compute(make_input(), path);

  const auto output = controller.compute(make_input(2.2, 1.0), path);

  ASSERT_TRUE(output.valid);
  EXPECT_EQ(output.target_index, 1U);
  EXPECT_DOUBLE_EQ(output.target.x, 2.0);
  EXPECT_DOUBLE_EQ(output.pursuit_target.x, 2.0);
  EXPECT_DOUBLE_EQ(output.pursuit_target.y, 0.0);
  EXPECT_FALSE(output.goal_reached);
}

TEST(PurePursuitController, StandardPursuitRaisesOnlyNonzeroLinearCommands)
{
  PurePursuitController controller(enhanced_config());
  const std::vector<Point2D> path{{0.0, 0.0}, {2.0, 0.0}};
  controller.compute(make_input(), path);

  const auto near_goal = controller.compute(make_input(1.5, 0.0), path);
  ASSERT_TRUE(near_goal.valid);
  EXPECT_GT(near_goal.raw_linear_velocity, 0.0);
  EXPECT_LT(near_goal.raw_linear_velocity, 0.5);
  EXPECT_DOUBLE_EQ(near_goal.linear_velocity, 0.5);
  EXPECT_TRUE(near_goal.minimum_linear_applied);
  EXPECT_DOUBLE_EQ(near_goal.angular_velocity, 0.0);

  const auto goal = controller.compute(make_input(1.90, 0.0), path);
  EXPECT_TRUE(goal.goal_reached);
  EXPECT_DOUBLE_EQ(goal.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(goal.angular_velocity, 0.0);
}

TEST(PurePursuitController, StandardPursuitUsesStationaryTurnFloorAndHysteresis)
{
  PurePursuitController controller(enhanced_config());
  const std::vector<Point2D> path{{0.0, 0.0}, {0.0, 2.0}};

  const auto initial = controller.compute(make_input(), path);
  ASSERT_TRUE(initial.valid);
  EXPECT_TRUE(initial.turning_in_place);
  EXPECT_TRUE(initial.turning_breakaway_active);
  EXPECT_DOUBLE_EQ(initial.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(initial.angular_velocity, 1.5);

  const auto moving = controller.compute(make_input(0.0, 0.0, 1.0), path);
  EXPECT_TRUE(moving.turning_in_place);
  EXPECT_FALSE(moving.turning_breakaway_active);
  EXPECT_DOUBLE_EQ(moving.angular_velocity, 1.5);

  const auto aligned = controller.compute(make_input(0.0, 0.0, 1.40), path);
  EXPECT_FALSE(aligned.turning_in_place);
  EXPECT_DOUBLE_EQ(aligned.linear_velocity, 0.5);
}

TEST(PurePursuitController, StandardPursuitKeepsSubthresholdAndExactZeroSteeringAtZero)
{
  PurePursuitController controller(enhanced_config());
  const std::vector<Point2D> path{{0.0, 0.0}, {10.0, 0.0}};
  controller.compute(make_input(), path);

  const auto exact = controller.compute(make_input(1.0, 0.0), path);
  EXPECT_DOUBLE_EQ(exact.raw_angular_velocity, 0.0);
  EXPECT_DOUBLE_EQ(exact.angular_velocity, 0.0);

  const auto small = controller.compute(make_input(2.0, 0.02), path);
  ASSERT_LT(std::abs(small.raw_angular_velocity), 0.05);
  EXPECT_DOUBLE_EQ(small.angular_velocity, 0.0);
  EXPECT_FALSE(small.minimum_angular_applied);
}

TEST(PurePursuitController, StandardPursuitAppliesMovingYawFloorAfterRawOmegaDeadband)
{
  PurePursuitController controller(enhanced_config());
  const std::vector<Point2D> path{{0.0, 0.0}, {10.0, 0.0}};
  controller.compute(make_input(), path);

  const auto output = controller.compute(make_input(1.0, 0.2), path);

  ASSERT_TRUE(output.valid);
  EXPECT_LT(output.raw_angular_velocity, -0.05);
  EXPECT_DOUBLE_EQ(output.angular_velocity, -1.0);
  EXPECT_TRUE(output.minimum_angular_applied);
}

TEST(PurePursuitController, StandardPursuitRequiresZeroFrameBeforeSteeringReversal)
{
  PurePursuitController controller(enhanced_config());
  const std::vector<Point2D> path{{0.0, 0.0}, {10.0, 0.0}};
  controller.compute(make_input(), path);

  EXPECT_DOUBLE_EQ(
    controller.compute(make_input(1.0, -0.2), path).angular_velocity, 1.0);
  EXPECT_DOUBLE_EQ(
    controller.compute(make_input(2.0, 0.2), path).angular_velocity, 0.0);
  EXPECT_DOUBLE_EQ(
    controller.compute(make_input(2.0, 0.2), path).angular_velocity, -1.0);
}

TEST(PurePursuitController, StandardPursuitKeepsNinetyDegreeCornersOrdered)
{
  for (const double direction : {-1.0, 1.0}) {
    ControllerConfig config = enhanced_config();
    config.lookahead_min_m = 2.0;
    config.lookahead_max_m = 2.0;
    PurePursuitController controller(config);
    const std::vector<Point2D> path{
      {0.0, 0.0}, {5.0, 0.0}, {5.0, direction * 5.0}};
    controller.compute(make_input(), path);

    const auto approach = controller.compute(make_input(4.0, 0.0), path);

    ASSERT_TRUE(approach.valid);
    EXPECT_EQ(approach.target_index, 1U);
    EXPECT_EQ(approach.pursuit_segment_index, 1U);
    EXPECT_DOUBLE_EQ(approach.pursuit_target.x, 5.0);
    EXPECT_DOUBLE_EQ(approach.pursuit_target.y, 0.0);
    EXPECT_DOUBLE_EQ(approach.curvature, 0.0);
    EXPECT_FALSE(approach.turning_in_place);
    EXPECT_DOUBLE_EQ(approach.angular_velocity, 0.0);

    const auto corner = controller.compute(make_input(5.0, 0.0), path);
    EXPECT_EQ(corner.target_index, 2U);
    EXPECT_TRUE(corner.turning_in_place);
    EXPECT_DOUBLE_EQ(corner.linear_velocity, 0.0);
    EXPECT_DOUBLE_EQ(corner.angular_velocity, direction * 1.5);
  }
}

TEST(PurePursuitController, StandardPursuitHandlesRepeatedWaypoints)
{
  PurePursuitController controller(enhanced_config());
  const std::vector<Point2D> path{
    {0.0, 0.0}, {0.0, 0.0}, {2.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}};

  const auto first = controller.compute(make_input(), path);
  ASSERT_TRUE(first.valid);
  EXPECT_EQ(first.target_index, 2U);
  EXPECT_TRUE(std::isfinite(first.pursuit_target.x));
  EXPECT_TRUE(std::isfinite(first.curvature));

  const auto second = controller.compute(make_input(2.0, 0.0), path);
  ASSERT_TRUE(second.valid);
  EXPECT_EQ(second.target_index, 4U);
  EXPECT_FALSE(second.goal_reached);

  const auto goal = controller.compute(make_input(4.0, 0.0), path);
  EXPECT_TRUE(goal.goal_reached);
  EXPECT_DOUBLE_EQ(goal.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(goal.angular_velocity, 0.0);
}

TEST(PurePursuitController, StandardPursuitWrapsTrackingErrorAcrossPi)
{
  PurePursuitController controller(enhanced_config());
  const std::vector<Point2D> path{{0.0, 0.0}, {-2.0, -0.002}};

  const auto output = controller.compute(
    make_input(0.0, 0.0, kPi - 0.001), path);

  ASSERT_TRUE(output.valid);
  EXPECT_FALSE(output.turning_in_place);
  EXPECT_NEAR(output.path_yaw, -kPi + 0.001, 1.0e-6);
  EXPECT_NEAR(output.yaw_error, 0.002, 1.0e-6);
  EXPECT_DOUBLE_EQ(output.angular_velocity, 0.0);
}

TEST(PurePursuitController, StandardPursuitKeepsStationaryTurnFloorAfterMeasuredYawChange)
{
  PurePursuitController controller(enhanced_config());
  const double target_yaw = 0.80;
  const std::vector<Point2D> path{
    {0.0, 0.0}, {2.0 * std::cos(target_yaw), 2.0 * std::sin(target_yaw)}};

  EXPECT_DOUBLE_EQ(controller.compute(make_input(), path).angular_velocity, 1.5);

  const auto below = controller.compute(make_input(0.0, 0.0, 0.049), path);
  EXPECT_TRUE(below.turning_breakaway_active);
  EXPECT_DOUBLE_EQ(below.angular_velocity, 1.5);

  const auto above = controller.compute(make_input(0.0, 0.0, 0.051), path);
  EXPECT_FALSE(above.turning_breakaway_active);
  EXPECT_DOUBLE_EQ(above.angular_velocity, 1.5);

  const auto repeated = controller.compute(make_input(0.0, 0.0, 0.051), path);
  EXPECT_TRUE(repeated.turning_in_place);
  EXPECT_DOUBLE_EQ(repeated.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(repeated.angular_velocity, 1.5);
}

TEST(PurePursuitController, FieldRouteTurnsStraddleConfiguredRawOmegaDeadband)
{
  ControllerConfig config = enhanced_config();
  config.lookahead_min_m = 2.0;
  config.lookahead_max_m = 2.0;

  const auto command_at_corner = [&config](const double turn_rad) {
      PurePursuitController controller(config);
      const std::vector<Point2D> path{
        {0.0, 0.0}, {10.0, 0.0},
        {10.0 + 10.0 * std::cos(turn_rad), 10.0 * std::sin(turn_rad)}};
      controller.compute(make_input(), path);
      return controller.compute(make_input(10.0, 0.0), path);
    };

  const auto largest_turn = command_at_corner(6.47174 * kPi / 180.0);
  EXPECT_GT(largest_turn.raw_angular_velocity, 0.05);
  EXPECT_DOUBLE_EQ(largest_turn.angular_velocity, 1.0);

  const auto smaller_turn = command_at_corner(4.68950 * kPi / 180.0);
  EXPECT_LT(smaller_turn.raw_angular_velocity, 0.05);
  EXPECT_DOUBLE_EQ(smaller_turn.angular_velocity, 0.0);
}

TEST(PurePursuitController, StandardPursuitFollowsFieldRouteInKinematicModel)
{
  ControllerConfig config = enhanced_config();
  config.nominal_speed = 0.50;
  config.max_speed = 1.00;
  config.max_yaw_rate = 1.50;
  config.max_curvature = 1.00;
  config.lookahead_min_m = 1.5;
  config.lookahead_max_m = 3.0;
  config.lookahead_speed_gain = 1.0;
  config.slowdown_distance = 1.20;
  config.waypoint_tolerance = 0.50;
  config.goal_tolerance = 0.50;
  PurePursuitController controller(config);
  const std::vector<Point2D> path{
    {0.0, 0.0}, {24.306349, -4.5342}, {51.398388, -9.823478},
    {77.730579, -18.135508}, {98.74537, -22.921271}};
  ControlInput input = make_input();
  constexpr double time_step = 0.02;
  std::size_t greatest_target_index = 0U;
  bool reached_goal = false;
  int completion_step = 20000;
  double travelled_distance = 0.0;
  double maximum_cross_track = 0.0;
  int steering_reversals = 0;
  double previous_steering_direction = 0.0;

  for (int step = 0; step < 20000; ++step) {
    const auto output = controller.compute(input, path);
    ASSERT_TRUE(output.valid);
    EXPECT_GE(output.target_index, greatest_target_index);
    greatest_target_index = output.target_index;
    EXPECT_GE(output.linear_velocity, 0.0);
    EXPECT_LE(output.linear_velocity, config.max_speed);
    EXPECT_LE(std::abs(output.angular_velocity), config.max_yaw_rate);
    if (output.goal_reached) {
      reached_goal = true;
      completion_step = step;
      break;
    }

    maximum_cross_track = std::max(
      maximum_cross_track, std::abs(output.cross_track_error));
    travelled_distance += output.linear_velocity * time_step;
    if (!output.turning_in_place && output.angular_velocity != 0.0) {
      const double direction = std::copysign(1.0, output.angular_velocity);
      if (previous_steering_direction != 0.0 &&
        direction != previous_steering_direction)
      {
        ++steering_reversals;
      }
      previous_steering_direction = direction;
    }

    input.pose.x += output.linear_velocity * std::cos(input.pose.yaw) * time_step;
    input.pose.y += output.linear_velocity * std::sin(input.pose.yaw) * time_step;
    input.pose.yaw = std::atan2(
      std::sin(input.pose.yaw + output.angular_velocity * time_step),
      std::cos(input.pose.yaw + output.angular_velocity * time_step));
  }

  EXPECT_TRUE(reached_goal);
  EXPECT_EQ(greatest_target_index, path.size() - 1U);
  EXPECT_LT(completion_step, 13000);
  EXPECT_LT(maximum_cross_track, 1.0);
  EXPECT_LT(travelled_distance, 112.0);
  EXPECT_LT(steering_reversals, 50);
}

}  // namespace
