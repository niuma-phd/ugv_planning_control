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

TrackingYawPulseShaper::TrackingYawPulseShaper(
  const double publish_rate_hz,
  const double minimum_yaw_rate,
  const double minimum_pulse_duration_sec)
{
  set_config(publish_rate_hz, minimum_yaw_rate, minimum_pulse_duration_sec);
}

void TrackingYawPulseShaper::set_config(
  const double publish_rate_hz,
  const double minimum_yaw_rate,
  const double minimum_pulse_duration_sec) noexcept
{
  publish_rate_hz_ = publish_rate_hz;
  minimum_yaw_rate_ = minimum_yaw_rate;
  minimum_pulse_duration_sec_ = minimum_pulse_duration_sec;
  if (config_is_valid()) {
    minimum_pulse_ticks_ = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
        std::ceil(publish_rate_hz_ * minimum_pulse_duration_sec_)));
  } else {
    minimum_pulse_ticks_ = 1U;
  }
  reset();
}

bool TrackingYawPulseShaper::config_is_valid() const noexcept
{
  return std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0 &&
         std::isfinite(minimum_yaw_rate_) && minimum_yaw_rate_ > 0.0 &&
         std::isfinite(minimum_pulse_duration_sec_) &&
         minimum_pulse_duration_sec_ > 0.0;
}

double TrackingYawPulseShaper::step(
  const double desired_yaw_rate,
  const bool correction_active) noexcept
{
  if (!config_is_valid() || !correction_active ||
    !std::isfinite(desired_yaw_rate) ||
    std::abs(desired_yaw_rate) <= 0.0)
  {
    reset();
    return 0.0;
  }

  const double magnitude = std::abs(desired_yaw_rate);
  if (magnitude >= minimum_yaw_rate_) {
    reset();
    return desired_yaw_rate;
  }

  const double direction = std::copysign(1.0, desired_yaw_rate);
  if (direction != pulse_direction_) {
    pulse_accumulator_ = 0.0;
    remaining_pulse_ticks_ = 0U;
    pulse_direction_ = direction;
  }

  pulse_accumulator_ += magnitude / minimum_yaw_rate_;
  if (remaining_pulse_ticks_ > 0U) {
    --remaining_pulse_ticks_;
    return pulse_direction_ * minimum_yaw_rate_;
  }

  if (pulse_accumulator_ + 1.0e-12 >=
    static_cast<double>(minimum_pulse_ticks_))
  {
    pulse_accumulator_ -= static_cast<double>(minimum_pulse_ticks_);
    remaining_pulse_ticks_ = minimum_pulse_ticks_ - 1U;
    return pulse_direction_ * minimum_yaw_rate_;
  }
  return 0.0;
}

void TrackingYawPulseShaper::reset() noexcept
{
  pulse_accumulator_ = 0.0;
  pulse_direction_ = 0.0;
  remaining_pulse_ticks_ = 0U;
}

PurePursuitController::PurePursuitController(const ControllerConfig & config)
: config_(config)
{
}

void PurePursuitController::set_config(const ControllerConfig & config)
{
  config_ = config;
  projection_target_index_ = static_cast<std::size_t>(-1);
  active_segment_progress_m_ = 0.0;
  reset_control_state();
}

const ControllerConfig & PurePursuitController::config() const noexcept
{
  return config_;
}

void PurePursuitController::reset_progress() noexcept
{
  active_waypoint_index_ = 0U;
  goal_latched_ = false;
  projection_target_index_ = static_cast<std::size_t>(-1);
  active_segment_progress_m_ = 0.0;
  reset_control_state();
}

bool PurePursuitController::config_is_valid() const noexcept
{
  const bool legacy_parameters_valid =
    std::isfinite(config_.nominal_speed) && config_.nominal_speed >= 0.0 &&
         std::isfinite(config_.max_speed) && config_.max_speed > 0.0 &&
         std::isfinite(config_.max_yaw_rate) && config_.max_yaw_rate > 0.0 &&
         std::isfinite(config_.max_curvature) && config_.max_curvature > 0.0 &&
         std::isfinite(config_.turn_in_place_threshold_rad) &&
         config_.turn_in_place_threshold_rad > 0.0 &&
         config_.turn_in_place_threshold_rad <= kPi / 2.0 &&
         std::isfinite(config_.slowdown_distance) && config_.slowdown_distance > 0.0 &&
         std::isfinite(config_.waypoint_tolerance) && config_.waypoint_tolerance > 0.0 &&
         std::isfinite(config_.goal_tolerance) && config_.goal_tolerance > 0.0;
  if (!legacy_parameters_valid || !config_.enhanced_tracking_enabled) {
    return legacy_parameters_valid;
  }

  return std::isfinite(config_.minimum_linear_speed) &&
         config_.minimum_linear_speed > 0.0 &&
         config_.minimum_linear_speed <= config_.max_speed &&
         std::isfinite(config_.minimum_tracking_yaw_rate) &&
         config_.minimum_tracking_yaw_rate > 0.0 &&
         config_.minimum_tracking_yaw_rate <= config_.max_yaw_rate &&
         std::isfinite(config_.minimum_turning_yaw_rate) &&
         config_.minimum_turning_yaw_rate > 0.0 &&
         config_.minimum_turning_yaw_rate <= config_.max_yaw_rate &&
         std::isfinite(config_.lookahead_min_m) && config_.lookahead_min_m > 0.0 &&
         std::isfinite(config_.lookahead_max_m) &&
         config_.lookahead_max_m >= config_.lookahead_min_m &&
         std::isfinite(config_.lookahead_speed_gain) &&
         config_.lookahead_speed_gain >= 0.0 &&
         std::isfinite(config_.turning_motion_threshold_rad) &&
         config_.turning_motion_threshold_rad > 0.0 &&
         config_.turning_motion_threshold_rad <=
         config_.turn_in_place_exit_threshold_rad &&
         std::isfinite(config_.turn_in_place_exit_threshold_rad) &&
         config_.turn_in_place_exit_threshold_rad > 0.0 &&
         config_.turn_in_place_exit_threshold_rad <
         config_.turn_in_place_threshold_rad &&
         std::isfinite(config_.tracking_omega_enter_threshold_rad_s) &&
         config_.tracking_omega_enter_threshold_rad_s > 0.0 &&
         config_.tracking_omega_enter_threshold_rad_s <= config_.max_yaw_rate &&
         std::isfinite(config_.tracking_omega_exit_threshold_rad_s) &&
         config_.tracking_omega_exit_threshold_rad_s >= 0.0 &&
         config_.tracking_omega_exit_threshold_rad_s <
         config_.tracking_omega_enter_threshold_rad_s;
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

double PurePursuitController::wrap_angle(double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double PurePursuitController::signed_floor(
  const double value, const double minimum_magnitude) noexcept
{
  if (value == 0.0) {
    return 0.0;
  }
  return std::copysign(std::max(std::abs(value), minimum_magnitude), value);
}

bool PurePursuitController::pursuit_geometry(
  const Pose2D & pose, const std::vector<Point2D> & path,
  const std::size_t target_index, const double lookahead_distance,
  Point2D & projection, Point2D & pursuit_target,
  std::size_t & pursuit_segment_index, double & path_yaw,
  double & reference_yaw, double & cross_track_error) noexcept
{
  const Point2D current{pose.x, pose.y};
  if (target_index == 0U) {
    projection = current;
    pursuit_target = path.front();
    pursuit_segment_index = 0U;
    path_yaw = std::atan2(
      pursuit_target.y - pose.y, pursuit_target.x - pose.x);
    reference_yaw = path_yaw;
    cross_track_error = 0.0;
    return distance(current, pursuit_target) > std::sqrt(kMinimumSquaredDistance);
  }

  std::size_t start_index = target_index;
  const Point2D & finish = path[target_index];
  while (start_index > 0U) {
    --start_index;
    const double candidate_x = finish.x - path[start_index].x;
    const double candidate_y = finish.y - path[start_index].y;
    if (candidate_x * candidate_x + candidate_y * candidate_y >
      kMinimumSquaredDistance)
    {
      if (projection_target_index_ != target_index) {
        projection_target_index_ = target_index;
        active_segment_progress_m_ = 0.0;
      }

      const Point2D & start = path[start_index];
      const double segment_x = finish.x - start.x;
      const double segment_y = finish.y - start.y;
      const double segment_length = std::hypot(segment_x, segment_y);
      const double unit_x = segment_x / segment_length;
      const double unit_y = segment_y / segment_length;
      const double relative_x = current.x - start.x;
      const double relative_y = current.y - start.y;
      const double raw_projection_m = relative_x * unit_x + relative_y * unit_y;
      const double clamped_projection_m = std::clamp(
        raw_projection_m, 0.0, segment_length);
      active_segment_progress_m_ = std::max(
        active_segment_progress_m_, clamped_projection_m);
      projection = Point2D{
        start.x + unit_x * active_segment_progress_m_,
        start.y + unit_y * active_segment_progress_m_};
      cross_track_error =
        unit_x * (current.y - projection.y) -
        unit_y * (current.x - projection.x);

      // If the vehicle has gone beyond an unconfirmed checkpoint while still
      // outside its capture corridor, do not let preview skip that CSV row.
      if (raw_projection_m >= segment_length &&
        distance(current, finish) > config_.waypoint_tolerance &&
        !passed_segment(pose, start, finish, config_.waypoint_tolerance))
      {
        pursuit_target = finish;
        pursuit_segment_index = target_index;
        path_yaw = std::atan2(segment_y, segment_x);
        reference_yaw = std::atan2(
          pursuit_target.y - pose.y, pursuit_target.x - pose.x);
        return true;
      }

      double distance_to_advance = lookahead_distance;
      const double current_segment_remaining =
        segment_length - active_segment_progress_m_;
      if (distance_to_advance <= current_segment_remaining) {
        pursuit_target = Point2D{
          projection.x + unit_x * distance_to_advance,
          projection.y + unit_y * distance_to_advance};
        pursuit_segment_index = target_index;
        path_yaw = std::atan2(segment_y, segment_x);
      } else {
        // CSV rows are mandatory ordered checkpoints. Do not preview onto a
        // future segment until the current row has actually been confirmed;
        // a hairpin could otherwise put the pursuit point behind the vehicle
        // or even back on its current pose and permanently skip this row.
        pursuit_target = finish;
        pursuit_segment_index = target_index;
        path_yaw = std::atan2(segment_y, segment_x);
      }

      const double pursuit_dx = pursuit_target.x - pose.x;
      const double pursuit_dy = pursuit_target.y - pose.y;
      if (pursuit_dx * pursuit_dx + pursuit_dy * pursuit_dy <=
        kMinimumSquaredDistance)
      {
        reference_yaw = path_yaw;
      } else {
        reference_yaw = std::atan2(pursuit_dy, pursuit_dx);
      }
      return true;
    }
  }

  projection = current;
  pursuit_target = finish;
  pursuit_segment_index = target_index;
  path_yaw = std::atan2(finish.y - pose.y, finish.x - pose.x);
  reference_yaw = path_yaw;
  cross_track_error = 0.0;
  return distance(current, finish) > std::sqrt(kMinimumSquaredDistance);
}

void PurePursuitController::reset_control_state() noexcept
{
  turning_in_place_latched_ = false;
  turning_yaw_direction_ = 0.0;
  turning_start_yaw_ = 0.0;
  turning_motion_confirmed_ = false;
  tracking_yaw_correction_latched_ = false;
  tracking_yaw_direction_ = 0.0;
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
    reset_control_state();
    return output;
  }

  if (active_waypoint_index_ >= path.size()) {
    reset_progress();
  }

  if (goal_latched_) {
    reset_control_state();
    output.target = path.back();
    output.target_index = path.size() - 1U;
    output.valid = true;
    output.goal_reached = true;
    return output;
  }

  const Point2D current{input.pose.x, input.pose.y};
  const std::size_t previous_waypoint_index = active_waypoint_index_;

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
  if (active_waypoint_index_ != previous_waypoint_index) {
    reset_control_state();
  }

  const double goal_distance = distance(current, path.back());
  if (active_waypoint_index_ == path.size() - 1U &&
    goal_distance <= config_.goal_tolerance)
  {
    goal_latched_ = true;
    reset_control_state();
    output.target = path.back();
    output.target_index = path.size() - 1U;
    output.valid = true;
    output.goal_reached = true;
    return output;
  }

  const std::size_t target_index = active_waypoint_index_;
  const Point2D target = path[target_index];
  const double cos_yaw = std::cos(input.pose.yaw);
  const double sin_yaw = std::sin(input.pose.yaw);

  output.target = target;
  output.target_index = target_index;
  output.valid = true;

  if (!config_.enhanced_tracking_enabled) {
    reset_control_state();
    const double dx = target.x - input.pose.x;
    const double dy = target.y - input.pose.y;
    const double target_x_base = cos_yaw * dx + sin_yaw * dy;
    const double target_y_base = -sin_yaw * dx + cos_yaw * dy;
    const double squared_distance =
      target_x_base * target_x_base + target_y_base * target_y_base;
    output.path_projection = current;
    output.pursuit_target = target;
    output.pursuit_segment_index = target_index;
    output.lookahead_distance = std::sqrt(squared_distance);
    if (squared_distance <= kMinimumSquaredDistance) {
      return output;
    }

    const double target_heading = std::atan2(target_y_base, target_x_base);
    const double raw_curvature = 2.0 * target_y_base / squared_distance;
    output.curvature = std::clamp(
      raw_curvature, -config_.max_curvature, config_.max_curvature);
    if (target_x_base <= 0.0 ||
      std::abs(target_heading) >= config_.turn_in_place_threshold_rad)
    {
      output.turning_in_place = true;
      output.raw_angular_velocity = target_heading;
      output.angular_velocity = std::clamp(
        target_heading, -config_.max_yaw_rate, config_.max_yaw_rate);
      return output;
    }

    const double remaining = remaining_length(input.pose, path, target_index);
    const double end_scale = std::clamp(
      remaining / config_.slowdown_distance, 0.0, 1.0);
    const double heading_scale = std::clamp(std::cos(target_heading), 0.0, 1.0);
    output.raw_linear_velocity = std::clamp(
      config_.nominal_speed * end_scale * heading_scale, 0.0, config_.max_speed);
    output.linear_velocity = output.raw_linear_velocity;

    // Preserve the requested curvature when the yaw-rate limit is active. A
    // sharp turn slows down instead of independently clipping omega and
    // cutting inside the route.
    if (std::abs(output.curvature) > 0.0) {
      output.linear_velocity = std::min(
        output.linear_velocity, config_.max_yaw_rate / std::abs(output.curvature));
    }
    output.raw_angular_velocity = output.linear_velocity * output.curvature;
    output.angular_velocity = output.raw_angular_velocity;
    return output;
  }

  const double remaining = remaining_length(input.pose, path, target_index);
  const double end_scale = std::clamp(
    remaining / config_.slowdown_distance, 0.0, 1.0);
  const double lookahead_speed_basis = std::clamp(
    config_.nominal_speed * end_scale, 0.0, config_.max_speed);
  output.lookahead_distance = std::clamp(
    config_.lookahead_min_m +
    config_.lookahead_speed_gain * lookahead_speed_basis,
    config_.lookahead_min_m, config_.lookahead_max_m);
  if (!pursuit_geometry(
      input.pose, path, target_index, output.lookahead_distance,
      output.path_projection, output.pursuit_target,
      output.pursuit_segment_index, output.path_yaw, output.reference_yaw,
      output.cross_track_error))
  {
    output.valid = false;
    reset_control_state();
    return output;
  }

  const double pursuit_dx = output.pursuit_target.x - input.pose.x;
  const double pursuit_dy = output.pursuit_target.y - input.pose.y;
  const double pursuit_x_base = cos_yaw * pursuit_dx + sin_yaw * pursuit_dy;
  const double pursuit_y_base = -sin_yaw * pursuit_dx + cos_yaw * pursuit_dy;
  const double pursuit_distance_squared =
    pursuit_x_base * pursuit_x_base + pursuit_y_base * pursuit_y_base;
  if (pursuit_distance_squared <= kMinimumSquaredDistance) {
    output.valid = false;
    reset_control_state();
    return output;
  }

  const double pursuit_heading = std::atan2(pursuit_y_base, pursuit_x_base);
  output.yaw_error = wrap_angle(output.reference_yaw - input.pose.yaw);
  output.curvature = std::clamp(
    2.0 * pursuit_y_base / pursuit_distance_squared,
    -config_.max_curvature, config_.max_curvature);

  const double checkpoint_dx = target.x - input.pose.x;
  const double checkpoint_dy = target.y - input.pose.y;
  const double checkpoint_x_base =
    cos_yaw * checkpoint_dx + sin_yaw * checkpoint_dy;
  const double checkpoint_y_base =
    -sin_yaw * checkpoint_dx + cos_yaw * checkpoint_dy;
  const double checkpoint_heading = std::atan2(
    checkpoint_y_base, checkpoint_x_base);
  double alignment_yaw_error = checkpoint_heading;
  if (checkpoint_x_base > 0.0 &&
    std::abs(checkpoint_heading) < config_.turn_in_place_threshold_rad &&
    pursuit_x_base <= 0.0)
  {
    alignment_yaw_error = pursuit_heading;
  }

  const double absolute_alignment_error = std::abs(alignment_yaw_error);
  bool turning_direction_reversal_blocked = false;
  if (turning_in_place_latched_) {
    if (!turning_motion_confirmed_ &&
      std::abs(wrap_angle(input.pose.yaw - turning_start_yaw_)) >=
      config_.turning_motion_threshold_rad)
    {
      turning_motion_confirmed_ = true;
    }
    const bool crossed_reference_near_zero =
      turning_yaw_direction_ != 0.0 &&
      alignment_yaw_error * turning_yaw_direction_ <= 0.0 &&
      absolute_alignment_error <= config_.turn_in_place_threshold_rad;
    if (crossed_reference_near_zero)
    {
      turning_yaw_direction_ = alignment_yaw_error == 0.0 ? 0.0 :
        std::copysign(1.0, alignment_yaw_error);
      turning_start_yaw_ = input.pose.yaw;
      turning_motion_confirmed_ = false;
      turning_direction_reversal_blocked = true;
    } else if (checkpoint_x_base > 0.0 && pursuit_x_base > 0.0 &&
      absolute_alignment_error <= config_.turn_in_place_exit_threshold_rad)
    {
      turning_in_place_latched_ = false;
      turning_yaw_direction_ = 0.0;
      turning_motion_confirmed_ = false;
    }
  } else if (checkpoint_x_base <= 0.0 || pursuit_x_base <= 0.0 ||
    absolute_alignment_error >= config_.turn_in_place_threshold_rad)
  {
    turning_in_place_latched_ = true;
    turning_yaw_direction_ = std::copysign(1.0, alignment_yaw_error);
    turning_start_yaw_ = input.pose.yaw;
    turning_motion_confirmed_ = false;
  }

  if (turning_in_place_latched_) {
    tracking_yaw_correction_latched_ = false;
    tracking_yaw_direction_ = 0.0;
    output.turning_in_place = true;
    output.yaw_correction_active = true;
    output.turning_breakaway_active = !turning_motion_confirmed_;
    output.reference_yaw = wrap_angle(input.pose.yaw + alignment_yaw_error);
    output.yaw_error = alignment_yaw_error;
    output.raw_angular_velocity = alignment_yaw_error;
    if (turning_direction_reversal_blocked || turning_yaw_direction_ == 0.0) {
      return output;
    }
    const double directed_raw_angular_velocity =
      turning_yaw_direction_ * std::abs(output.raw_angular_velocity);
    const double minimum_yaw_rate = config_.minimum_turning_yaw_rate;
    output.angular_velocity = signed_floor(
      directed_raw_angular_velocity, minimum_yaw_rate);
    output.minimum_angular_applied = std::abs(output.raw_angular_velocity) <
      minimum_yaw_rate;
    output.angular_velocity = std::clamp(
      output.angular_velocity, -config_.max_yaw_rate, config_.max_yaw_rate);
    return output;
  }

  const double heading_scale = std::clamp(std::cos(pursuit_heading), 0.0, 1.0);
  output.raw_linear_velocity = std::clamp(
    config_.nominal_speed * end_scale * heading_scale, 0.0, config_.max_speed);
  output.linear_velocity = output.raw_linear_velocity;

  if (std::abs(output.curvature) > 0.0) {
    output.linear_velocity = std::min(
      output.linear_velocity, config_.max_yaw_rate / std::abs(output.curvature));
  }
  if (output.linear_velocity > 0.0 &&
    output.linear_velocity < config_.minimum_linear_speed)
  {
    output.linear_velocity = config_.minimum_linear_speed;
    output.minimum_linear_applied = true;
  }
  output.linear_velocity = std::clamp(
    output.linear_velocity, 0.0, config_.max_speed);

  // Standard Pure Pursuit tracking law. Path-yaw information is diagnostic;
  // TRACK steering is determined only by the interpolated pursuit point.
  output.raw_angular_velocity = output.linear_velocity * output.curvature;

  const double steering_direction_source = output.raw_angular_velocity;
  bool direction_reversal_blocked = false;
  if (tracking_yaw_correction_latched_) {
    const bool correction_inside_exit_deadband =
      std::abs(output.raw_angular_velocity) <=
      config_.tracking_omega_exit_threshold_rad_s;
    if (correction_inside_exit_deadband ||
      steering_direction_source * tracking_yaw_direction_ <= 0.0)
    {
      tracking_yaw_correction_latched_ = false;
      tracking_yaw_direction_ = 0.0;
      direction_reversal_blocked = true;
    }
  }
  if (!tracking_yaw_correction_latched_ && !direction_reversal_blocked &&
    std::abs(output.raw_angular_velocity) >=
    config_.tracking_omega_enter_threshold_rad_s &&
    steering_direction_source != 0.0)
  {
    tracking_yaw_correction_latched_ = true;
    tracking_yaw_direction_ = std::copysign(1.0, steering_direction_source);
  }

  output.yaw_correction_active = tracking_yaw_correction_latched_;
  if (tracking_yaw_correction_latched_) {
    double angular_magnitude = std::abs(output.raw_angular_velocity);
    if (angular_magnitude < config_.minimum_tracking_yaw_rate) {
      angular_magnitude = config_.minimum_tracking_yaw_rate;
      output.minimum_angular_applied = true;
    }
    output.angular_velocity = tracking_yaw_direction_ * angular_magnitude;
    output.angular_velocity = std::clamp(
      output.angular_velocity, -config_.max_yaw_rate, config_.max_yaw_rate);
  }
  return output;
}

}  // namespace ugv_subject2_mvp
