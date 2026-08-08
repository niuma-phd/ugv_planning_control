#ifndef UGV_SUBJECT2_MVP__PURE_PURSUIT_CONTROLLER_HPP_
#define UGV_SUBJECT2_MVP__PURE_PURSUIT_CONTROLLER_HPP_

#include <cstddef>
#include <vector>

namespace ugv_subject2_mvp
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ControllerConfig
{
  double nominal_speed{0.50};
  double max_speed{1.00};
  double max_yaw_rate{0.8};
  double max_curvature{1.5};
  double turn_in_place_threshold_rad{1.0471975511965976};
  double slowdown_distance{1.5};
  double waypoint_tolerance{0.30};
  double goal_tolerance{0.15};
  bool enhanced_tracking_enabled{false};
  double minimum_linear_speed{0.50};
  double minimum_tracking_yaw_rate{1.00};
  double minimum_turning_yaw_rate{1.50};
  double lookahead_min_m{1.50};
  double lookahead_max_m{3.00};
  double lookahead_speed_gain{1.00};
  double turning_motion_threshold_rad{0.05};
  double turn_in_place_exit_threshold_rad{0.20};
  double tracking_omega_enter_threshold_rad_s{0.05};
  double tracking_omega_exit_threshold_rad_s{0.02};
};

struct ControlInput
{
  Pose2D pose;
  bool inputs_valid{false};
};

struct ControlOutput
{
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double raw_linear_velocity{0.0};
  double raw_angular_velocity{0.0};
  double curvature{0.0};
  double path_yaw{0.0};
  double reference_yaw{0.0};
  double yaw_error{0.0};
  double cross_track_error{0.0};
  Point2D target;
  Point2D path_projection;
  Point2D pursuit_target;
  std::size_t target_index{0U};
  std::size_t pursuit_segment_index{0U};
  double lookahead_distance{0.0};
  bool valid{false};
  bool goal_reached{false};
  bool turning_in_place{false};
  bool turning_breakaway_active{false};
  bool yaw_correction_active{false};
  bool minimum_linear_applied{false};
  bool minimum_angular_applied{false};
};

class TrackingYawPulseShaper
{
public:
  TrackingYawPulseShaper(
    double publish_rate_hz = 20.0,
    double minimum_yaw_rate = 1.0,
    double minimum_pulse_duration_sec = 0.10);

  void set_config(
    double publish_rate_hz,
    double minimum_yaw_rate,
    double minimum_pulse_duration_sec) noexcept;
  bool config_is_valid() const noexcept;
  double step(double desired_yaw_rate, bool correction_active) noexcept;
  void reset() noexcept;

private:
  double publish_rate_hz_{20.0};
  double minimum_yaw_rate_{1.0};
  double minimum_pulse_duration_sec_{0.10};
  double pulse_accumulator_{0.0};
  double pulse_direction_{0.0};
  std::size_t minimum_pulse_ticks_{2U};
  std::size_t remaining_pulse_ticks_{0U};
};

class PurePursuitController
{
public:
  explicit PurePursuitController(const ControllerConfig & config = ControllerConfig{});

  void set_config(const ControllerConfig & config);
  const ControllerConfig & config() const noexcept;
  bool config_is_valid() const noexcept;
  void reset_progress() noexcept;
  ControlOutput compute(const ControlInput & input, const std::vector<Point2D> & path);

private:
  static bool finite(const Point2D & point) noexcept;
  static double distance(const Point2D & first, const Point2D & second) noexcept;
  static bool passed_segment(
    const Pose2D & pose, const Point2D & start, const Point2D & finish,
    double tolerance) noexcept;
  static double wrap_angle(double angle) noexcept;
  static double signed_floor(double value, double minimum_magnitude) noexcept;
  bool pursuit_geometry(
    const Pose2D & pose, const std::vector<Point2D> & path,
    std::size_t target_index, double lookahead_distance,
    Point2D & projection, Point2D & pursuit_target,
    std::size_t & pursuit_segment_index, double & path_yaw,
    double & reference_yaw, double & cross_track_error) noexcept;
  double remaining_length(
    const Pose2D & pose, const std::vector<Point2D> & path,
    std::size_t target_index) const;
  void reset_control_state() noexcept;

  ControllerConfig config_;
  std::size_t active_waypoint_index_{0U};
  bool goal_latched_{false};
  bool turning_in_place_latched_{false};
  double turning_yaw_direction_{0.0};
  double turning_start_yaw_{0.0};
  bool turning_motion_confirmed_{false};
  bool tracking_yaw_correction_latched_{false};
  double tracking_yaw_direction_{0.0};
  std::size_t projection_target_index_{static_cast<std::size_t>(-1)};
  double active_segment_progress_m_{0.0};
};

}  // namespace ugv_subject2_mvp

#endif  // UGV_SUBJECT2_MVP__PURE_PURSUIT_CONTROLLER_HPP_
