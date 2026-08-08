#include "ugv_subject2_mvp/stall_boost_controller.hpp"

#include <algorithm>
#include <cmath>

namespace ugv_subject2_mvp
{

namespace
{
constexpr double kCommandEpsilon = 1.0e-9;
}

const char * stall_boost_mode_name(const StallBoostMode mode) noexcept
{
  switch (mode) {
    case StallBoostMode::none:
      return "NONE";
    case StallBoostMode::track:
      return "TRACK";
    case StallBoostMode::align:
      return "ALIGN";
  }
  return "UNKNOWN";
}

const char * stall_boost_phase_name(const StallBoostPhase phase) noexcept
{
  switch (phase) {
    case StallBoostPhase::idle:
      return "IDLE";
    case StallBoostPhase::observing:
      return "OBSERVING";
    case StallBoostPhase::boosting:
      return "BOOSTING";
    case StallBoostPhase::ramp_down:
      return "RAMP_DOWN";
    case StallBoostPhase::cooldown:
      return "COOLDOWN";
    case StallBoostPhase::suppressed:
      return "SUPPRESSED";
  }
  return "UNKNOWN";
}

StallBoostController::StallBoostController(const StallBoostConfig & config)
: config_(config)
{
}

void StallBoostController::set_config(const StallBoostConfig & config) noexcept
{
  config_ = config;
  reset();
}

const StallBoostConfig & StallBoostController::config() const noexcept
{
  return config_;
}

bool StallBoostController::config_is_valid() const noexcept
{
  return std::isfinite(config_.detection_duration_sec) &&
         config_.detection_duration_sec > 0.0 &&
         std::isfinite(config_.maximum_observation_gap_sec) &&
         config_.maximum_observation_gap_sec > 0.0 &&
         std::isfinite(config_.motion_translation_threshold_m) &&
         config_.motion_translation_threshold_m > 0.0 &&
         std::isfinite(config_.motion_yaw_threshold_rad) &&
         config_.motion_yaw_threshold_rad > 0.0 &&
         std::isfinite(config_.minimum_linear_command_mps) &&
         config_.minimum_linear_command_mps >= 0.0 &&
         std::isfinite(config_.minimum_angular_command_rad_s) &&
         config_.minimum_angular_command_rad_s >= 0.0 &&
         std::isfinite(config_.linear_boost_speed_mps) &&
         config_.linear_boost_speed_mps > config_.minimum_linear_command_mps &&
         std::isfinite(config_.angular_boost_rate_rad_s) &&
         config_.angular_boost_rate_rad_s > config_.minimum_angular_command_rad_s &&
         std::isfinite(config_.boost_min_duration_sec) &&
         config_.boost_min_duration_sec >= 0.0 &&
         std::isfinite(config_.boost_max_duration_sec) &&
         config_.boost_max_duration_sec >= config_.boost_min_duration_sec &&
         std::isfinite(config_.ramp_down_sec) && config_.ramp_down_sec > 0.0 &&
         std::isfinite(config_.cooldown_sec) && config_.cooldown_sec >= 0.0 &&
         config_.max_attempts > 0;
}

double StallBoostController::wrap_angle(const double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double StallBoostController::direction(const double value) noexcept
{
  if (std::abs(value) <= kCommandEpsilon) {
    return 0.0;
  }
  return std::copysign(1.0, value);
}

bool StallBoostController::finite_pose(const Pose2D & pose) noexcept
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

void StallBoostController::reset() noexcept
{
  mode_ = StallBoostMode::none;
  phase_ = StallBoostPhase::idle;
  anchor_pose_ = Pose2D{};
  phase_started_sec_ = 0.0;
  last_step_sec_ = 0.0;
  maximum_translation_excursion_m_ = 0.0;
  maximum_yaw_excursion_rad_ = 0.0;
  command_direction_ = 0.0;
  target_index_ = 0U;
  attempt_count_ = 0;
  context_initialized_ = false;
}

void StallBoostController::begin_phase(
  const StallBoostPhase phase, const double now_sec, const Pose2D & pose) noexcept
{
  phase_ = phase;
  phase_started_sec_ = now_sec;
  anchor_pose_ = pose;
  maximum_translation_excursion_m_ = 0.0;
  maximum_yaw_excursion_rad_ = 0.0;
}

void StallBoostController::update_excursion(const Pose2D & pose) noexcept
{
  maximum_translation_excursion_m_ = std::max(
    maximum_translation_excursion_m_,
    std::hypot(pose.x - anchor_pose_.x, pose.y - anchor_pose_.y));
  maximum_yaw_excursion_rad_ = std::max(
    maximum_yaw_excursion_rad_,
    std::abs(wrap_angle(pose.yaw - anchor_pose_.yaw)));
}

bool StallBoostController::observed_motion() const noexcept
{
  if (mode_ == StallBoostMode::align) {
    return maximum_yaw_excursion_rad_ >= config_.motion_yaw_threshold_rad;
  }
  if (mode_ == StallBoostMode::track) {
    // TRACK exists to make forward progress. Yaw-only motion must not hide a
    // chassis that is still unable to translate under a nonzero linear command.
    return maximum_translation_excursion_m_ >=
           config_.motion_translation_threshold_m;
  }
  return false;
}

bool StallBoostController::recovered_motion() const noexcept
{
  if (mode_ == StallBoostMode::align) {
    return maximum_yaw_excursion_rad_ >= config_.motion_yaw_threshold_rad;
  }
  if (mode_ == StallBoostMode::track) {
    return maximum_translation_excursion_m_ >=
           config_.motion_translation_threshold_m;
  }
  return false;
}

void StallBoostController::apply_boost(
  const StallBoostInput & input, const double blend,
  StallBoostOutput & output) const noexcept
{
  const double bounded_blend = std::clamp(blend, 0.0, 1.0);
  if (mode_ == StallBoostMode::track) {
    const double planned = std::abs(input.desired_linear_velocity);
    const double floor = config_.linear_boost_speed_mps +
      (planned - config_.linear_boost_speed_mps) * bounded_blend;
    output.linear_velocity = std::copysign(
      std::max(planned, floor), input.desired_linear_velocity);
    // Preserve Pure Pursuit steering while breaking forward stiction.
    output.angular_velocity = input.desired_angular_velocity;
  } else if (mode_ == StallBoostMode::align) {
    const double planned = std::abs(input.desired_angular_velocity);
    const double floor = config_.angular_boost_rate_rad_s +
      (planned - config_.angular_boost_rate_rad_s) * bounded_blend;
    output.linear_velocity = 0.0;
    output.angular_velocity = std::copysign(
      std::max(planned, floor), input.desired_angular_velocity);
  }
  output.boost_applied =
    std::abs(output.linear_velocity - input.desired_linear_velocity) >
    kCommandEpsilon ||
    std::abs(output.angular_velocity - input.desired_angular_velocity) >
    kCommandEpsilon;
}

StallBoostOutput StallBoostController::step(const StallBoostInput & input) noexcept
{
  StallBoostOutput output;
  output.linear_velocity = input.desired_linear_velocity;
  output.angular_velocity = input.desired_angular_velocity;
  output.previous_phase = phase_;

  const bool finite_input = std::isfinite(input.steady_time_sec) &&
    finite_pose(input.pose) && std::isfinite(input.desired_linear_velocity) &&
    std::isfinite(input.desired_angular_velocity);
  if (!input.command_valid || !finite_input) {
    reset();
    output.linear_velocity = 0.0;
    output.angular_velocity = 0.0;
    output.phase = phase_;
    output.mode = mode_;
    output.phase_changed = output.previous_phase != output.phase;
    return output;
  }

  if (!config_.enabled) {
    reset();
    output.phase = phase_;
    output.mode = mode_;
    output.phase_changed = output.previous_phase != output.phase;
    return output;
  }

  StallBoostMode requested_mode = StallBoostMode::none;
  double requested_direction = 0.0;
  if (input.turning_in_place &&
    std::abs(input.desired_angular_velocity) >=
    config_.minimum_angular_command_rad_s)
  {
    requested_mode = StallBoostMode::align;
    requested_direction = direction(input.desired_angular_velocity);
  } else if (!input.turning_in_place &&
    std::abs(input.desired_linear_velocity) >= config_.minimum_linear_command_mps)
  {
    requested_mode = StallBoostMode::track;
    requested_direction = direction(input.desired_linear_velocity);
  }

  if (requested_mode == StallBoostMode::none || requested_direction == 0.0) {
    reset();
    output.phase = phase_;
    output.mode = mode_;
    output.phase_changed = output.previous_phase != output.phase;
    return output;
  }

  const bool time_reversed = context_initialized_ &&
    input.steady_time_sec <= last_step_sec_;
  const bool observation_gap_too_large = context_initialized_ &&
    input.steady_time_sec - last_step_sec_ >
    config_.maximum_observation_gap_sec;
  const bool context_changed = !context_initialized_ || time_reversed ||
    observation_gap_too_large ||
    requested_mode != mode_ || input.target_index != target_index_ ||
    requested_direction != command_direction_;
  if (context_changed) {
    mode_ = requested_mode;
    target_index_ = input.target_index;
    command_direction_ = requested_direction;
    attempt_count_ = 0;
    context_initialized_ = true;
    begin_phase(StallBoostPhase::observing, input.steady_time_sec, input.pose);
    last_step_sec_ = input.steady_time_sec;
    output.mode = mode_;
    output.phase = phase_;
    output.phase_changed = output.previous_phase != output.phase;
    return output;
  }

  last_step_sec_ = input.steady_time_sec;
  update_excursion(input.pose);
  // Preserve evidence from the phase that made the transition. begin_phase()
  // resets its anchor, but the transition log must show the triggering motion.
  const double measured_translation_excursion_m =
    maximum_translation_excursion_m_;
  const double measured_yaw_excursion_rad = maximum_yaw_excursion_rad_;
  const double phase_age_sec = input.steady_time_sec - phase_started_sec_;

  switch (phase_) {
    case StallBoostPhase::idle:
      begin_phase(StallBoostPhase::observing, input.steady_time_sec, input.pose);
      break;

    case StallBoostPhase::observing:
      if (phase_age_sec >= config_.detection_duration_sec) {
        if (observed_motion()) {
          attempt_count_ = 0;
          begin_phase(StallBoostPhase::observing, input.steady_time_sec, input.pose);
        } else if (attempt_count_ >= config_.max_attempts) {
          begin_phase(StallBoostPhase::suppressed, input.steady_time_sec, input.pose);
        } else {
          ++attempt_count_;
          begin_phase(StallBoostPhase::boosting, input.steady_time_sec, input.pose);
        }
      }
      break;

    case StallBoostPhase::boosting:
      if (phase_age_sec >= config_.boost_min_duration_sec && recovered_motion()) {
        attempt_count_ = 0;
        begin_phase(StallBoostPhase::ramp_down, input.steady_time_sec, input.pose);
      } else if (phase_age_sec >= config_.boost_max_duration_sec) {
        begin_phase(StallBoostPhase::ramp_down, input.steady_time_sec, input.pose);
      }
      break;

    case StallBoostPhase::ramp_down:
      if (phase_age_sec >= config_.ramp_down_sec) {
        begin_phase(StallBoostPhase::cooldown, input.steady_time_sec, input.pose);
      }
      break;

    case StallBoostPhase::cooldown:
      if (phase_age_sec >= config_.cooldown_sec) {
        begin_phase(StallBoostPhase::observing, input.steady_time_sec, input.pose);
      }
      break;

    case StallBoostPhase::suppressed:
      if (observed_motion()) {
        attempt_count_ = 0;
        begin_phase(StallBoostPhase::observing, input.steady_time_sec, input.pose);
      }
      break;
  }

  if (phase_ == StallBoostPhase::boosting) {
    apply_boost(input, 0.0, output);
  } else if (phase_ == StallBoostPhase::ramp_down) {
    const double ramp_age_sec = input.steady_time_sec - phase_started_sec_;
    apply_boost(input, ramp_age_sec / config_.ramp_down_sec, output);
  }

  output.translation_excursion_m = measured_translation_excursion_m;
  output.yaw_excursion_rad = measured_yaw_excursion_rad;
  output.attempt_count = attempt_count_;
  output.mode = mode_;
  output.phase = phase_;
  output.phase_changed = output.previous_phase != output.phase;
  return output;
}

}  // namespace ugv_subject2_mvp
