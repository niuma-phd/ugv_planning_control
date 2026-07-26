#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace ugv_localization_mvp
{

struct RecoveryPose
{
  std::int64_t stamp_ns{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct RecoveryCoreSettings
{
  std::size_t gps_sample_count{5U};
  std::size_t odom_sample_count{5U};
  double max_gps_position_covariance{4.0};
  double max_gps_yaw_covariance{0.25};
  double max_gps_window_position_span_m{0.75};
  double max_gps_window_yaw_span_rad{0.35};
  double max_odom_window_position_span_m{0.30};
  double max_odom_window_yaw_span_rad{0.20};
  double max_anchor_correction_m{10.0};
  double max_anchor_correction_yaw_rad{1.57};
};

bool validRecoverySettings(const RecoveryCoreSettings & settings);
bool initialOdomFaultConfirmed(
  bool odom_valid, bool odom_was_ever_valid, double invalid_age_sec,
  double startup_invalid_grace_sec);
bool recoveryPoseFromGps(
  const geometry_msgs::msg::PoseWithCovariance & pose, std::int64_t stamp_ns,
  const RecoveryCoreSettings & settings, RecoveryPose & result, std::string & reason);
bool recoveryPoseFromOdom(
  const geometry_msgs::msg::Pose & pose, std::int64_t stamp_ns,
  RecoveryPose & result, std::string & reason);

class StablePoseWindow
{
public:
  StablePoseWindow(
    std::size_t required_samples, double max_position_span_m, double max_yaw_span_rad);
  bool add(const RecoveryPose & sample);
  void clear();
  bool ready() const;
  const RecoveryPose & latest() const;
  std::size_t size() const;

private:
  bool stable() const;
  std::size_t required_samples_;
  double max_position_span_m_;
  double max_yaw_span_rad_;
  std::deque<RecoveryPose> samples_;
};

geometry_msgs::msg::Transform mapOdomFromRecovery(
  const RecoveryPose & map_base_gps, const RecoveryPose & odom_base);
RecoveryPose transformPose(
  const geometry_msgs::msg::Transform & parent_child, const RecoveryPose & child_pose);
bool anchorCorrectionAcceptable(
  const RecoveryPose & gps_pose, const RecoveryPose & last_trusted_odom,
  const geometry_msgs::msg::Transform & map_odom,
  const RecoveryCoreSettings & settings, std::string & reason);
bool transformsNear(
  const geometry_msgs::msg::Transform & first,
  const geometry_msgs::msg::Transform & second,
  double translation_tolerance_m, double yaw_tolerance_rad);
double wrappedAngleDistance(double first, double second);

}  // namespace ugv_localization_mvp
