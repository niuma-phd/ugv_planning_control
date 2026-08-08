#ifndef UGV_SUBJECT2_MVP__STALL_BOOST_CONTROLLER_HPP_
#define UGV_SUBJECT2_MVP__STALL_BOOST_CONTROLLER_HPP_

#include <cstddef>

#include "ugv_subject2_mvp/pure_pursuit_controller.hpp"

namespace ugv_subject2_mvp
{

enum class StallBoostMode
{
  none,
  track,
  align
};

enum class StallBoostPhase
{
  idle,
  observing,
  boosting,
  ramp_down,
  cooldown,
  suppressed
};

const char * stall_boost_mode_name(StallBoostMode mode) noexcept;
const char * stall_boost_phase_name(StallBoostPhase phase) noexcept;

struct StallBoostConfig
{
  bool enabled{false};
  double detection_duration_sec{0.80};
  double maximum_observation_gap_sec{0.30};
  double motion_translation_threshold_m{0.05};
  double motion_yaw_threshold_rad{0.03};
  double minimum_linear_command_mps{0.30};
  double minimum_angular_command_rad_s{0.50};
  double linear_boost_speed_mps{2.00};
  double angular_boost_rate_rad_s{1.50};
  double boost_min_duration_sec{0.40};
  double boost_max_duration_sec{1.00};
  double ramp_down_sec{0.80};
  double cooldown_sec{1.00};
  int max_attempts{3};
};

struct StallBoostInput
{
  double steady_time_sec{0.0};
  Pose2D pose;
  double desired_linear_velocity{0.0};
  double desired_angular_velocity{0.0};
  std::size_t target_index{0U};
  bool command_valid{false};
  bool turning_in_place{false};
};

struct StallBoostOutput
{
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double translation_excursion_m{0.0};
  double yaw_excursion_rad{0.0};
  int attempt_count{0};
  StallBoostMode mode{StallBoostMode::none};
  StallBoostPhase previous_phase{StallBoostPhase::idle};
  StallBoostPhase phase{StallBoostPhase::idle};
  bool phase_changed{false};
  bool boost_applied{false};
};

class StallBoostController
{
public:
  explicit StallBoostController(const StallBoostConfig & config = StallBoostConfig{});

  void set_config(const StallBoostConfig & config) noexcept;
  const StallBoostConfig & config() const noexcept;
  bool config_is_valid() const noexcept;
  StallBoostOutput step(const StallBoostInput & input) noexcept;
  void reset() noexcept;

private:
  static double wrap_angle(double angle) noexcept;
  static double direction(double value) noexcept;
  static bool finite_pose(const Pose2D & pose) noexcept;

  void begin_phase(
    StallBoostPhase phase, double now_sec, const Pose2D & pose) noexcept;
  void update_excursion(const Pose2D & pose) noexcept;
  bool observed_motion() const noexcept;
  bool recovered_motion() const noexcept;
  void apply_boost(
    const StallBoostInput & input, double blend, StallBoostOutput & output) const noexcept;

  StallBoostConfig config_;
  StallBoostMode mode_{StallBoostMode::none};
  StallBoostPhase phase_{StallBoostPhase::idle};
  Pose2D anchor_pose_;
  double phase_started_sec_{0.0};
  double last_step_sec_{0.0};
  double maximum_translation_excursion_m_{0.0};
  double maximum_yaw_excursion_rad_{0.0};
  double command_direction_{0.0};
  std::size_t target_index_{0U};
  int attempt_count_{0};
  bool context_initialized_{false};
};

}  // namespace ugv_subject2_mvp

#endif  // UGV_SUBJECT2_MVP__STALL_BOOST_CONTROLLER_HPP_
