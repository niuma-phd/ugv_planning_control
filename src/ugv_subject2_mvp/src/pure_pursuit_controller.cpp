#include "ugv_subject2_mvp/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>

namespace ugv_subject2_mvp
{

namespace
{
constexpr double kMinimumSquaredDistance = 1.0e-8;
constexpr double kPi = 3.14159265358979323846;
}

PurePursuitController::PurePursuitController(const ControllerConfig & config)
: config_(config)
{
}

void PurePursuitController::set_config(const ControllerConfig & config)
{
  config_ = config;
}

const ControllerConfig & PurePursuitController::config() const noexcept
{
  return config_;
}

void PurePursuitController::reset_progress() noexcept
{
  active_waypoint_index_ = 0U;
  goal_latched_ = false;
}

bool PurePursuitController::config_is_valid() const noexcept
{
  return std::isfinite(config_.nominal_speed) && config_.nominal_speed >= 0.0 &&
         std::isfinite(config_.max_speed) && config_.max_speed > 0.0 &&
         std::isfinite(config_.max_yaw_rate) && config_.max_yaw_rate > 0.0 &&
         std::isfinite(config_.max_curvature) && config_.max_curvature > 0.0 &&
         std::isfinite(config_.turn_in_place_threshold_rad) &&
         config_.turn_in_place_threshold_rad > 0.0 &&
         config_.turn_in_place_threshold_rad <= kPi / 2.0 &&
         std::isfinite(config_.slowdown_distance) && config_.slowdown_distance > 0.0 &&
         std::isfinite(config_.waypoint_tolerance) && config_.waypoint_tolerance > 0.0 &&
         std::isfinite(config_.goal_tolerance) && config_.goal_tolerance > 0.0;
}

bool PurePursuitController::finite(const Point2D & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

double PurePursuitController::distance(const Point2D & first, const Point2D & second) noexcept
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

bool PurePursuitController::passed_segment(
  const Pose2D & pose, const Point2D & start, const Point2D & finish,
  const double tolerance) noexcept
{
  const Point2D current{pose.x, pose.y};
  if (distance(current, finish) <= tolerance) {
    return true;
  }

  const double segment_x = finish.x - start.x;
  const double segment_y = finish.y - start.y;
  const double length_squared = segment_x * segment_x + segment_y * segment_y;
  if (length_squared <= kMinimumSquaredDistance) {
    return true;
  }

  const double relative_x = current.x - start.x;
  const double relative_y = current.y - start.y;
  const double projection =
    (relative_x * segment_x + relative_y * segment_y) / length_squared;
  if (projection < 1.0) {
    return false;
  }

  const double cross_track = std::abs(
    segment_x * relative_y - segment_y * relative_x) / std::sqrt(length_squared);
  return cross_track <= tolerance;
}

double PurePursuitController::remaining_length(
  const Pose2D & pose, const std::vector<Point2D> & path,
  const std::size_t target_index) const
{
  const Point2D current{pose.x, pose.y};
  double remaining = distance(current, path[target_index]);
  for (std::size_t index = target_index + 1U; index < path.size(); ++index) {
    remaining += distance(path[index - 1U], path[index]);
  }
  return remaining;
}

ControlOutput PurePursuitController::compute(
  const ControlInput & input, const std::vector<Point2D> & path)
{
  ControlOutput output;
  if (!input.inputs_valid || !config_is_valid() || path.empty() ||
    !std::isfinite(input.pose.x) || !std::isfinite(input.pose.y) ||
    !std::isfinite(input.pose.yaw) ||
    !std::all_of(path.begin(), path.end(), finite))
  {
    return output;
  }

  if (active_waypoint_index_ >= path.size()) {
    reset_progress();
  }

  if (goal_latched_) {
    output.target = path.back();
    output.target_index = path.size() - 1U;
    output.valid = true;
    output.goal_reached = true;
    return output;
  }

  const Point2D current{input.pose.x, input.pose.y};

  // The CSV is an ordered queue, not an unordered point cloud. Confirm only
  // the active waypoint; never search for a globally nearest future point.
  while (active_waypoint_index_ < path.size() - 1U) {
    const bool active_reached = active_waypoint_index_ == 0U ?
      distance(current, path.front()) <= config_.waypoint_tolerance :
      passed_segment(
        input.pose, path[active_waypoint_index_ - 1U],
        path[active_waypoint_index_], config_.waypoint_tolerance);
    if (!active_reached) {
      break;
    }
    ++active_waypoint_index_;
  }

  const double goal_distance = distance(current, path.back());
  if (active_waypoint_index_ == path.size() - 1U &&
    goal_distance <= config_.goal_tolerance)
  {
    goal_latched_ = true;
    output.target = path.back();
    output.target_index = path.size() - 1U;
    output.valid = true;
    output.goal_reached = true;
    return output;
  }

  const std::size_t target_index = active_waypoint_index_;
  const Point2D target = path[target_index];
  const double dx = target.x - input.pose.x;
  const double dy = target.y - input.pose.y;
  const double cos_yaw = std::cos(input.pose.yaw);
  const double sin_yaw = std::sin(input.pose.yaw);
  const double target_x_base = cos_yaw * dx + sin_yaw * dy;
  const double target_y_base = -sin_yaw * dx + cos_yaw * dy;
  const double squared_distance = target_x_base * target_x_base + target_y_base * target_y_base;

  output.target = target;
  output.target_index = target_index;
  output.valid = true;
  if (squared_distance <= kMinimumSquaredDistance) {
    return output;
  }

  const double target_heading = std::atan2(target_y_base, target_x_base);
  if (target_x_base <= 0.0 ||
    std::abs(target_heading) >= config_.turn_in_place_threshold_rad)
  {
    output.turning_in_place = true;
    output.angular_velocity = std::clamp(
      target_heading, -config_.max_yaw_rate, config_.max_yaw_rate);
    return output;
  }

  const double raw_curvature = 2.0 * target_y_base / squared_distance;
  output.curvature = std::clamp(
    raw_curvature, -config_.max_curvature, config_.max_curvature);

  const double remaining = remaining_length(input.pose, path, target_index);
  const double end_scale = std::clamp(remaining / config_.slowdown_distance, 0.0, 1.0);
  const double heading_scale = std::clamp(std::cos(target_heading), 0.0, 1.0);
  output.linear_velocity = std::clamp(
    config_.nominal_speed * end_scale * heading_scale, 0.0, config_.max_speed);

  // Preserve the requested curvature when the yaw-rate limit is active. A
  // sharp turn slows down instead of independently clipping omega and cutting
  // inside the route.
  if (std::abs(output.curvature) > 0.0) {
    output.linear_velocity = std::min(
      output.linear_velocity, config_.max_yaw_rate / std::abs(output.curvature));
  }
  output.angular_velocity = output.linear_velocity * output.curvature;
  return output;
}

}  // namespace ugv_subject2_mvp
