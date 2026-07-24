#pragma once

#include <string>
#include <vector>

namespace ugv_subject1_avoidance_mvp
{

bool frame_id_matches(const std::string & actual, const std::string & expected);

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PlannerConfig
{
  double speed_mps{0.10};
  double max_curvature{1.2};
  int curvature_samples{25};
  double horizon_m{3.0};
  double step_m{0.10};
  double footprint_half_length_m{0.55};
  double footprint_half_width_m{0.40};
  double inflation_m{0.20};
  double goal_distance_weight{1.0};
  double heading_weight{0.7};
  double curvature_weight{0.15};
  double clearance_weight{0.35};
  double max_clearance_reward_m{2.0};
};

struct PlanResult
{
  bool active{false};
  bool has_safe_trajectory{false};
  double speed_mps{0.0};
  double yaw_rate_radps{0.0};
  double curvature{0.0};
  std::vector<Pose2> trajectory;
};

class LocalAvoidance
{
public:
  explicit LocalAvoidance(const PlannerConfig & config);

  PlanResult plan(
    const std::vector<Point2> & obstacles, const Point2 & goal,
    bool inputs_fresh) const;

private:
  std::vector<Pose2> rollout(double curvature) const;
  bool collides(const Pose2 & pose, const Point2 & obstacle) const;
  double clearance(const Pose2 & pose, const Point2 & obstacle) const;
  double score(
    const std::vector<Pose2> & trajectory, const std::vector<Point2> & obstacles,
    const Point2 & goal, double curvature) const;

  PlannerConfig config_;
};

}  // namespace ugv_subject1_avoidance_mvp
