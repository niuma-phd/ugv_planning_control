#include "ugv_localization_mvp/recovery_core.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ugv_localization_mvp/transform_math.hpp"

namespace ugv_localization_mvp
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
bool finitePositive(double value) {return std::isfinite(value) && value > 0.0;}

bool finitePoseValues(const geometry_msgs::msg::Pose & pose)
{
  return finiteAndNormalized(pose) &&
         std::isfinite(tf2::getYaw(pose.orientation));
}
}  // namespace

double wrappedAngleDistance(double first, double second)
{
  return std::abs(std::remainder(first - second, 2.0 * kPi));
}

bool validRecoverySettings(const RecoveryCoreSettings & settings)
{
  return settings.gps_sample_count > 0U && settings.odom_sample_count > 0U &&
         finitePositive(settings.max_gps_position_covariance) &&
         finitePositive(settings.max_gps_yaw_covariance) &&
         finitePositive(settings.max_gps_window_position_span_m) &&
         finitePositive(settings.max_gps_window_yaw_span_rad) &&
         finitePositive(settings.max_odom_window_position_span_m) &&
         finitePositive(settings.max_odom_window_yaw_span_rad) &&
         finitePositive(settings.max_anchor_correction_m) &&
         finitePositive(settings.max_anchor_correction_yaw_rad);
}

bool initialOdomFaultConfirmed(
  bool odom_valid, bool odom_was_ever_valid, double invalid_age_sec,
  double startup_invalid_grace_sec)
{
  if (odom_valid || !std::isfinite(invalid_age_sec) || invalid_age_sec < 0.0 ||
    !finitePositive(startup_invalid_grace_sec))
  {
    return false;
  }
  return odom_was_ever_valid || invalid_age_sec >= startup_invalid_grace_sec;
}

bool recoveryPoseFromGps(
  const geometry_msgs::msg::PoseWithCovariance & pose, std::int64_t stamp_ns,
  const RecoveryCoreSettings & settings, RecoveryPose & result, std::string & reason)
{
  if (stamp_ns <= 0) {
    reason = "GPS timestamp must be positive";
    return false;
  }
  if (!finitePoseValues(pose.pose)) {
    reason = "GPS pose or yaw quaternion is invalid";
    return false;
  }
  for (const auto covariance : pose.covariance) {
    if (!std::isfinite(covariance)) {
      reason = "GPS covariance must be finite";
      return false;
    }
  }
  constexpr std::size_t kCovarianceIndices[] = {0U, 7U, 14U, 35U};
  for (const auto index : kCovarianceIndices) {
    if (pose.covariance[index] <= 0.0) {
      reason = "GPS position and yaw covariance must prove populated uncertainty";
      return false;
    }
  }
  if (pose.covariance[0] > settings.max_gps_position_covariance ||
    pose.covariance[7] > settings.max_gps_position_covariance ||
    pose.covariance[14] > settings.max_gps_position_covariance)
  {
    reason = "GPS position covariance exceeds the configured limit";
    return false;
  }
  if (pose.covariance[35] > settings.max_gps_yaw_covariance)
  {
    reason = "GPS yaw covariance does not prove a trustworthy yaw";
    return false;
  }
  result = RecoveryPose{
    stamp_ns, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
    tf2::getYaw(pose.pose.orientation)};
  return true;
}

bool recoveryPoseFromOdom(
  const geometry_msgs::msg::Pose & pose, std::int64_t stamp_ns,
  RecoveryPose & result, std::string & reason)
{
  if (stamp_ns <= 0) {
    reason = "odometry timestamp must be positive";
    return false;
  }
  if (!finitePoseValues(pose)) {
    reason = "odometry pose is invalid";
    return false;
  }
  result = RecoveryPose{
    stamp_ns, pose.position.x, pose.position.y, pose.position.z,
    tf2::getYaw(pose.orientation)};
  return true;
}

StablePoseWindow::StablePoseWindow(
  std::size_t required_samples, double max_position_span_m, double max_yaw_span_rad)
: required_samples_(required_samples),
  max_position_span_m_(max_position_span_m),
  max_yaw_span_rad_(max_yaw_span_rad)
{
  if (required_samples_ == 0U || !finitePositive(max_position_span_m_) ||
    !finitePositive(max_yaw_span_rad_))
  {
    throw std::invalid_argument("stable window settings must be finite and positive");
  }
}

bool StablePoseWindow::add(const RecoveryPose & sample)
{
  if (sample.stamp_ns <= 0 || !std::isfinite(sample.x) || !std::isfinite(sample.y) ||
    !std::isfinite(sample.z) || !std::isfinite(sample.yaw))
  {
    return false;
  }
  if (!samples_.empty() && sample.stamp_ns <= samples_.back().stamp_ns) {return false;}
  samples_.push_back(sample);
  while (samples_.size() > required_samples_) {samples_.pop_front();}
  return ready();
}

void StablePoseWindow::clear() {samples_.clear();}
bool StablePoseWindow::ready() const {return samples_.size() == required_samples_ && stable();}
const RecoveryPose & StablePoseWindow::latest() const {return samples_.back();}
std::size_t StablePoseWindow::size() const {return samples_.size();}

bool StablePoseWindow::stable() const
{
  if (samples_.empty()) {return false;}
  double min_x = samples_.front().x;
  double max_x = min_x;
  double min_y = samples_.front().y;
  double max_y = min_y;
  double min_z = samples_.front().z;
  double max_z = min_z;
  for (const auto & sample : samples_) {
    min_x = std::min(min_x, sample.x);
    max_x = std::max(max_x, sample.x);
    min_y = std::min(min_y, sample.y);
    max_y = std::max(max_y, sample.y);
    min_z = std::min(min_z, sample.z);
    max_z = std::max(max_z, sample.z);
  }
  if (std::hypot(std::hypot(max_x - min_x, max_y - min_y), max_z - min_z) >
    max_position_span_m_)
  {
    return false;
  }
  // A span is a bound across the complete window, not only a distance from
  // the first sample.  Pairwise wrapped distances also handle samples that
  // straddle -pi/+pi without treating them as a nearly 2*pi jump.
  for (auto first = samples_.cbegin(); first != samples_.cend(); ++first) {
    for (auto second = std::next(first); second != samples_.cend(); ++second) {
      if (wrappedAngleDistance(first->yaw, second->yaw) > max_yaw_span_rad_) {
        return false;
      }
    }
  }
  return true;
}

geometry_msgs::msg::Transform mapOdomFromRecovery(
  const RecoveryPose & map_base_gps, const RecoveryPose & odom_base)
{
  geometry_msgs::msg::Pose odom_pose;
  odom_pose.position.x = odom_base.x;
  odom_pose.position.y = odom_base.y;
  odom_pose.position.z = odom_base.z;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, odom_base.yaw);
  odom_pose.orientation = tf2::toMsg(quaternion);
  return mapOdomFromStart(
    map_base_gps.x, map_base_gps.y, map_base_gps.z, map_base_gps.yaw, odom_pose);
}

RecoveryPose transformPose(
  const geometry_msgs::msg::Transform & parent_child, const RecoveryPose & child_pose)
{
  if (!finiteAndNormalized(parent_child)) {
    throw std::invalid_argument("parent-child transform must be finite and normalized");
  }
  const double parent_yaw = tf2::getYaw(parent_child.rotation);
  const double cosine = std::cos(parent_yaw);
  const double sine = std::sin(parent_yaw);
  return RecoveryPose{
    child_pose.stamp_ns,
    parent_child.translation.x + cosine * child_pose.x - sine * child_pose.y,
    parent_child.translation.y + sine * child_pose.x + cosine * child_pose.y,
    parent_child.translation.z + child_pose.z,
    std::remainder(parent_yaw + child_pose.yaw, 2.0 * kPi)};
}

bool anchorCorrectionAcceptable(
  const RecoveryPose & gps_pose, const RecoveryPose & last_trusted_odom,
  const geometry_msgs::msg::Transform & map_odom,
  const RecoveryCoreSettings & settings, std::string & reason)
{
  if (!finiteAndNormalized(map_odom)) {
    reason = "current map->odom transform is invalid";
    return false;
  }
  const auto previous_map_pose = transformPose(map_odom, last_trusted_odom);
  const double position_delta = std::hypot(
    std::hypot(gps_pose.x - previous_map_pose.x, gps_pose.y - previous_map_pose.y),
    gps_pose.z - previous_map_pose.z);
  if (position_delta > settings.max_anchor_correction_m) {
    reason = "GPS correction from the last trusted odometry exceeds the position limit";
    return false;
  }
  if (wrappedAngleDistance(gps_pose.yaw, previous_map_pose.yaw) >
    settings.max_anchor_correction_yaw_rad)
  {
    reason = "GPS correction from the last trusted odometry exceeds the yaw limit";
    return false;
  }
  return true;
}

bool transformsNear(
  const geometry_msgs::msg::Transform & first,
  const geometry_msgs::msg::Transform & second,
  double translation_tolerance_m, double yaw_tolerance_rad)
{
  if (!finiteAndNormalized(first) || !finiteAndNormalized(second) ||
    !finitePositive(translation_tolerance_m) || !finitePositive(yaw_tolerance_rad))
  {
    return false;
  }
  const double translation = std::hypot(
    std::hypot(
      first.translation.x - second.translation.x,
      first.translation.y - second.translation.y),
    first.translation.z - second.translation.z);
  return translation <= translation_tolerance_m &&
         wrappedAngleDistance(tf2::getYaw(first.rotation), tf2::getYaw(second.rotation)) <=
         yaw_tolerance_rad;
}

}  // namespace ugv_localization_mvp
