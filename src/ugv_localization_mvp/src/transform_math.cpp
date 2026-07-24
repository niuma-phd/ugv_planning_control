#include "ugv_localization_mvp/transform_math.hpp"

#include <cmath>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ugv_localization_mvp
{
namespace
{
bool finite(double v) {return std::isfinite(v);}
bool quaternionValid(double x, double y, double z, double w, double tolerance)
{
  if (!finite(x) || !finite(y) || !finite(z) || !finite(w)) {return false;}
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  return norm > 1.0e-12 && std::abs(norm - 1.0) <= tolerance;
}
}  // namespace

bool finiteAndNormalized(const geometry_msgs::msg::Pose & pose, double tolerance)
{
  return finite(pose.position.x) && finite(pose.position.y) && finite(pose.position.z) &&
         quaternionValid(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, tolerance);
}

bool finiteAndNormalized(const geometry_msgs::msg::Transform & transform, double tolerance)
{
  return finite(transform.translation.x) && finite(transform.translation.y) &&
         finite(transform.translation.z) && quaternionValid(
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w, tolerance);
}

geometry_msgs::msg::Quaternion normalizedQuaternion(
  const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (!finite(norm) || norm <= 1.0e-12) {
    throw std::invalid_argument("quaternion must be finite and non-zero");
  }
  geometry_msgs::msg::Quaternion result;
  result.x = quaternion.x / norm;
  result.y = quaternion.y / norm;
  result.z = quaternion.z / norm;
  result.w = quaternion.w / norm;
  return result;
}

geometry_msgs::msg::Transform makeTransform(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Transform result;
  result.translation.x = x;
  result.translation.y = y;
  result.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  result.rotation = tf2::toMsg(q);
  return result;
}

geometry_msgs::msg::Transform odomBaseFromRawLidar(
  const geometry_msgs::msg::Pose & world_lidar,
  const geometry_msgs::msg::Transform & base_lidar)
{
  auto normalized_world_lidar = world_lidar;
  auto normalized_base_lidar = base_lidar;
  normalized_world_lidar.orientation = normalizedQuaternion(world_lidar.orientation);
  normalized_base_lidar.rotation = normalizedQuaternion(base_lidar.rotation);
  tf2::Transform t_world_lidar;
  tf2::Transform t_base_lidar;
  tf2::fromMsg(normalized_world_lidar, t_world_lidar);
  tf2::fromMsg(normalized_base_lidar, t_base_lidar);
  return tf2::toMsg(t_world_lidar * t_base_lidar.inverse());
}

geometry_msgs::msg::Transform mapOdomFromStart(
  double map_x, double map_y, double map_z, double map_yaw,
  const geometry_msgs::msg::Pose & odom_base)
{
  tf2::Transform t_map_base;
  t_map_base.setOrigin(tf2::Vector3(map_x, map_y, map_z));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, map_yaw);
  t_map_base.setRotation(q);
  auto normalized_odom_base = odom_base;
  normalized_odom_base.orientation = normalizedQuaternion(odom_base.orientation);
  tf2::Transform t_odom_base;
  tf2::fromMsg(normalized_odom_base, t_odom_base);
  return tf2::toMsg(t_map_base * t_odom_base.inverse());
}

}  // namespace ugv_localization_mvp
