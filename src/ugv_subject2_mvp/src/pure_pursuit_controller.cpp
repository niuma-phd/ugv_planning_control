#include "ugv_subject2_mvp/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ugv_subject2_mvp
{

namespace
{
constexpr double kMinimumSquaredDistance = 1.0e-8;
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
  last_nearest_index_ = 0U;
  progress_initialized_ = false;
}

bool PurePursuitController::config_is_valid() const noexcept
{
  return std::isfinite(config_.nominal_speed) && config_.nominal_speed >= 0.0 &&
         std::isfinite(config_.max_speed) && config_.max_speed > 0.0 &&
         std::isfinite(config_.max_yaw_rate) && config_.max_yaw_rate > 0.0 &&
         std::isfinite(config_.max_curvature) && config_.max_curvature > 0.0 &&
         std::isfinite(config_.lookahead_distance) && config_.lookahead_distance > 0.0 &&
         std::isfinite(config_.lookahead_speed_gain) && config_.lookahead_speed_gain >= 0.0 &&
         std::isfinite(config_.min_lookahead) && config_.min_lookahead > 0.0 &&
         std::isfinite(config_.max_lookahead) &&
         config_.max_lookahead >= config_.min_lookahead &&
         std::isfinite(config_.slowdown_distance) && config_.slowdown_distance > 0.0 &&
         std::isfinite(config_.goal_tolerance) && config_.goal_tolerance >= 0.0;
}

bool PurePursuitController::finite(const Point2D & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

double PurePursuitController::distance(const Point2D & first, const Point2D & second) noexcept
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

std::size_t PurePursuitController::find_nearest(const ControlInput & input) const
{
  std::size_t begin = 0U;
  std::size_t end = input.path.size();
  if (progress_initialized_) {
    begin = last_nearest_index_ > config_.progress_backtrack ?
      last_nearest_index_ - config_.progress_backtrack : 0U;
    const std::size_t remaining = input.path.size() - std::min(last_nearest_index_, input.path.size());
    const std::size_t ahead = std::min(config_.progress_search_ahead + 1U, remaining);
    end = std::min(input.path.size(), last_nearest_index_ + ahead);
  }

  const Point2D current{input.pose.x, input.pose.y};
  double best_distance = std::numeric_limits<double>::infinity();
  std::size_t best_index = begin;
  for (std::size_t index = begin; index < end; ++index) {
    const double candidate = distance(current, input.path[index]);
    if (candidate < best_distance) {
      best_distance = candidate;
      best_index = index;
    }
  }
  return best_index;
}

std::size_t PurePursuitController::find_target(
  const std::vector<Point2D> & path, const std::size_t nearest_index,
  const double lookahead_distance) const
{
  double travelled = 0.0;
  for (std::size_t index = nearest_index + 1U; index < path.size(); ++index) {
    travelled += distance(path[index - 1U], path[index]);
    if (travelled >= lookahead_distance) {
      return index;
    }
  }
  return path.size() - 1U;
}

double PurePursuitController::remaining_length(
  const Pose2D & pose, const std::vector<Point2D> & path,
  const std::size_t nearest_index) const
{
  if (path.size() == 1U) {
    return distance(Point2D{pose.x, pose.y}, path.front());
  }

  const Point2D current{pose.x, pose.y};
  const std::size_t first_segment = nearest_index > 0U ? nearest_index - 1U : 0U;
  const std::size_t last_segment = std::min(nearest_index, path.size() - 2U);
  double best_cross_track = std::numeric_limits<double>::infinity();
  double best_remaining = std::numeric_limits<double>::infinity();

  for (std::size_t segment = first_segment; segment <= last_segment; ++segment) {
    const Point2D & start = path[segment];
    const Point2D & finish = path[segment + 1U];
    const double segment_x = finish.x - start.x;
    const double segment_y = finish.y - start.y;
    const double length_squared = segment_x * segment_x + segment_y * segment_y;
    double fraction = 0.0;
    if (length_squared > kMinimumSquaredDistance) {
      fraction = std::clamp(
        ((current.x - start.x) * segment_x + (current.y - start.y) * segment_y) /
        length_squared, 0.0, 1.0);
    }
    const Point2D projection{
      start.x + fraction * segment_x,
      start.y + fraction * segment_y};
    const double cross_track = distance(current, projection);
    if (cross_track >= best_cross_track) {
      continue;
    }

    double remaining = cross_track + distance(projection, finish);
    for (std::size_t index = segment + 2U; index < path.size(); ++index) {
      remaining += distance(path[index - 1U], path[index]);
    }
    best_cross_track = cross_track;
    best_remaining = remaining;
  }
  return best_remaining;
}

ControlOutput PurePursuitController::compute(const ControlInput & input)
{
  ControlOutput output;
  if (!input.inputs_valid || !config_is_valid() || input.path.empty() ||
    !std::isfinite(input.pose.x) || !std::isfinite(input.pose.y) ||
    !std::isfinite(input.pose.yaw) || !std::isfinite(input.current_speed) ||
    !std::all_of(input.path.begin(), input.path.end(), finite))
  {
    return output;
  }

  const std::size_t nearest = find_nearest(input);
  last_nearest_index_ = std::max(last_nearest_index_, nearest);
  progress_initialized_ = true;
  const std::size_t progress_index = last_nearest_index_;

  output.nearest_index = progress_index;
  const double remaining = remaining_length(input.pose, input.path, progress_index);
  const double goal_distance = distance(
    Point2D{input.pose.x, input.pose.y}, input.path.back());
  if (goal_distance <= config_.goal_tolerance) {
    output.goal_reached = true;
    output.valid = true;
    output.target = input.path.back();
    output.target_index = input.path.size() - 1U;
    return output;
  }

  double lookahead = config_.lookahead_distance;
  if (config_.use_speed_scaled_lookahead) {
    lookahead += config_.lookahead_speed_gain * std::abs(input.current_speed);
  }
  lookahead = std::clamp(lookahead, config_.min_lookahead, config_.max_lookahead);

  const std::size_t target_index = find_target(input.path, progress_index, lookahead);
  const Point2D target = input.path[target_index];
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
  if (squared_distance <= kMinimumSquaredDistance || target_x_base < 0.0) {
    return output;
  }

  const double raw_curvature = 2.0 * target_y_base / squared_distance;
  output.curvature = std::clamp(
    raw_curvature, -config_.max_curvature, config_.max_curvature);

  const double end_scale = std::clamp(remaining / config_.slowdown_distance, 0.0, 1.0);
  output.linear_velocity = std::clamp(
    config_.nominal_speed * end_scale, 0.0, config_.max_speed);
  output.angular_velocity = std::clamp(
    output.linear_velocity * output.curvature,
    -config_.max_yaw_rate, config_.max_yaw_rate);
  return output;
}

}  // namespace ugv_subject2_mvp
