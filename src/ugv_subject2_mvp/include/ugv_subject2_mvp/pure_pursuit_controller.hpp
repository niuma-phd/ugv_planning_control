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
  double nominal_speed{0.10};
  double max_speed{0.20};
  double max_yaw_rate{0.8};
  double max_curvature{1.5};
  double lookahead_distance{1.0};
  bool use_speed_scaled_lookahead{true};
  double lookahead_speed_gain{0.5};
  double min_lookahead{0.6};
  double max_lookahead{2.0};
  double slowdown_distance{1.5};
  double goal_tolerance{0.15};
  std::size_t progress_search_ahead{200U};
  std::size_t progress_backtrack{3U};
};

struct ControlInput
{
  Pose2D pose;
  std::vector<Point2D> path;
  double current_speed{0.0};
  bool inputs_valid{false};
};

struct ControlOutput
{
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double curvature{0.0};
  Point2D target;
  std::size_t nearest_index{0U};
  std::size_t target_index{0U};
  bool valid{false};
  bool goal_reached{false};
};

class PurePursuitController
{
public:
  explicit PurePursuitController(const ControllerConfig & config = ControllerConfig{});

  void set_config(const ControllerConfig & config);
  const ControllerConfig & config() const noexcept;
  void reset_progress() noexcept;
  ControlOutput compute(const ControlInput & input);

private:
  bool config_is_valid() const noexcept;
  static bool finite(const Point2D & point) noexcept;
  static double distance(const Point2D & first, const Point2D & second) noexcept;
  std::size_t find_nearest(const ControlInput & input) const;
  std::size_t find_target(
    const std::vector<Point2D> & path, std::size_t nearest_index,
    double lookahead_distance) const;
  double remaining_length(
    const Pose2D & pose, const std::vector<Point2D> & path,
    std::size_t nearest_index) const;

  ControllerConfig config_;
  std::size_t last_nearest_index_{0U};
  bool progress_initialized_{false};
};

}  // namespace ugv_subject2_mvp

#endif  // UGV_SUBJECT2_MVP__PURE_PURSUIT_CONTROLLER_HPP_
