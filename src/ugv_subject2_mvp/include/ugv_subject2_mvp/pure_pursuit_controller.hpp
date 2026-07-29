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
  double curvature{0.0};
  Point2D target;
  std::size_t target_index{0U};
  bool valid{false};
  bool goal_reached{false};
  bool turning_in_place{false};
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
  double remaining_length(
    const Pose2D & pose, const std::vector<Point2D> & path,
    std::size_t target_index) const;

  ControllerConfig config_;
  std::size_t active_waypoint_index_{0U};
  bool goal_latched_{false};
};

}  // namespace ugv_subject2_mvp

#endif  // UGV_SUBJECT2_MVP__PURE_PURSUIT_CONTROLLER_HPP_
