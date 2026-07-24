#include "ugv_subject1_avoidance_mvp/local_avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ugv_subject1_avoidance_mvp
{
namespace
{
constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr int kMaxCurvatureSamples = 1001;
constexpr double kMaxTrajectorySamples = 10000.0;
constexpr double kMaxConfiguredRolloutWork = 200000.0;

bool finite(const Point2 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}
}  // namespace

bool frame_id_matches(const std::string & actual, const std::string & expected)
{
  return !expected.empty() && actual == expected;
}

LocalAvoidance::LocalAvoidance(const PlannerConfig & config)
: config_(config)
{
  const double trajectory_samples = std::ceil(config_.horizon_m / config_.step_m);
  const bool finite_config =
    std::isfinite(config_.speed_mps) &&
    std::isfinite(config_.max_curvature) &&
    std::isfinite(config_.horizon_m) &&
    std::isfinite(config_.step_m) &&
    std::isfinite(config_.footprint_half_length_m) &&
    std::isfinite(config_.footprint_half_width_m) &&
    std::isfinite(config_.inflation_m) &&
    std::isfinite(config_.goal_distance_weight) &&
    std::isfinite(config_.heading_weight) &&
    std::isfinite(config_.curvature_weight) &&
    std::isfinite(config_.clearance_weight) &&
    std::isfinite(config_.max_clearance_reward_m);
  if (!finite_config || config_.speed_mps <= 0.0 || config_.max_curvature <= 0.0 ||
    config_.curvature_samples < 2 || config_.curvature_samples > kMaxCurvatureSamples ||
    config_.horizon_m <= 0.0 || config_.step_m <= 0.0 ||
    config_.footprint_half_length_m <= 0.0 || config_.footprint_half_width_m <= 0.0 ||
    config_.step_m > 2.0 * (config_.footprint_half_length_m + config_.inflation_m) ||
    !std::isfinite(trajectory_samples) || trajectory_samples > kMaxTrajectorySamples ||
    trajectory_samples * static_cast<double>(config_.curvature_samples) >
    kMaxConfiguredRolloutWork ||
    config_.inflation_m < 0.0 || config_.goal_distance_weight < 0.0 ||
    config_.heading_weight < 0.0 || config_.curvature_weight < 0.0 ||
    config_.clearance_weight < 0.0 || config_.max_clearance_reward_m < 0.0)
  {
    throw std::invalid_argument("invalid local avoidance configuration");
  }
}

PlanResult LocalAvoidance::plan(
  const std::vector<Point2> & obstacles, const Point2 & goal, bool inputs_fresh) const
{
  PlanResult result;
  if (!inputs_fresh || !finite(goal) ||
    !std::all_of(obstacles.begin(), obstacles.end(), finite))
  {
    result.active = true;
    return result;
  }
  const auto nominal_trajectory = rollout(0.0);
  const bool relevant_obstacle = std::any_of(
    nominal_trajectory.begin(), nominal_trajectory.end(),
    [this, &obstacles](const Pose2 & pose) {
      return std::any_of(
        obstacles.begin(), obstacles.end(),
        [this, &pose](const Point2 & obstacle) {return collides(pose, obstacle);});
    });
  if (!relevant_obstacle) {
    return result;
  }

  result.active = true;
  double best_score = std::numeric_limits<double>::infinity();

  const int samples = config_.curvature_samples;
  for (int index = 0; index < samples; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(samples - 1);
    const double curvature = -config_.max_curvature + 2.0 * config_.max_curvature * ratio;
    auto trajectory = rollout(curvature);

    bool collision = false;
    for (const auto & pose : trajectory) {
      for (const auto & obstacle : obstacles) {
        if (collides(pose, obstacle)) {
          collision = true;
          break;
        }
      }
      if (collision) {
        break;
      }
    }
    if (collision) {
      continue;
    }

    const double candidate_score = score(trajectory, obstacles, goal, curvature);
    if (candidate_score < best_score) {
      best_score = candidate_score;
      result.has_safe_trajectory = true;
      result.speed_mps = config_.speed_mps;
      result.curvature = curvature;
      result.yaw_rate_radps = config_.speed_mps * curvature;
      result.trajectory = std::move(trajectory);
    }
  }
  return result;
}

std::vector<Pose2> LocalAvoidance::rollout(double curvature) const
{
  std::vector<Pose2> trajectory;
  const int steps = static_cast<int>(std::ceil(config_.horizon_m / config_.step_m));
  trajectory.reserve(static_cast<std::size_t>(steps) + 1U);
  trajectory.push_back(Pose2{});
  for (int index = 1; index <= steps; ++index) {
    const double distance = std::min(index * config_.step_m, config_.horizon_m);
    Pose2 pose;
    if (std::abs(curvature) < kEpsilon) {
      pose.x = distance;
    } else {
      pose.x = std::sin(curvature * distance) / curvature;
      pose.y = (1.0 - std::cos(curvature * distance)) / curvature;
      pose.yaw = curvature * distance;
    }
    trajectory.push_back(pose);
  }
  return trajectory;
}

bool LocalAvoidance::collides(const Pose2 & pose, const Point2 & obstacle) const
{
  const double dx = obstacle.x - pose.x;
  const double dy = obstacle.y - pose.y;
  const double local_x = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
  const double local_y = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
  return std::abs(local_x) <= config_.footprint_half_length_m + config_.inflation_m &&
         std::abs(local_y) <= config_.footprint_half_width_m + config_.inflation_m;
}

double LocalAvoidance::clearance(const Pose2 & pose, const Point2 & obstacle) const
{
  const double dx = obstacle.x - pose.x;
  const double dy = obstacle.y - pose.y;
  const double local_x = std::abs(std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy);
  const double local_y = std::abs(-std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy);
  const double outside_x = std::max(
    0.0, local_x - config_.footprint_half_length_m - config_.inflation_m);
  const double outside_y = std::max(
    0.0, local_y - config_.footprint_half_width_m - config_.inflation_m);
  return std::hypot(outside_x, outside_y);
}

double LocalAvoidance::score(
  const std::vector<Pose2> & trajectory, const std::vector<Point2> & obstacles,
  const Point2 & goal, double curvature) const
{
  const auto & end = trajectory.back();
  const double goal_distance = std::hypot(goal.x - end.x, goal.y - end.y);
  const double desired_heading = std::atan2(goal.y - end.y, goal.x - end.x);
  const double heading_error = std::abs(normalize_angle(desired_heading - end.yaw));

  double minimum_clearance = config_.max_clearance_reward_m;
  for (const auto & pose : trajectory) {
    for (const auto & obstacle : obstacles) {
      minimum_clearance = std::min(minimum_clearance, clearance(pose, obstacle));
    }
  }

  return config_.goal_distance_weight * goal_distance +
         config_.heading_weight * heading_error +
         config_.curvature_weight * std::abs(curvature) -
         config_.clearance_weight * minimum_clearance;
}

}  // namespace ugv_subject1_avoidance_mvp
