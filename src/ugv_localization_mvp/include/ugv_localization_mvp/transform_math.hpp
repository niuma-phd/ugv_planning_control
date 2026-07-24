#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace ugv_localization_mvp
{

bool finiteAndNormalized(const geometry_msgs::msg::Pose & pose, double tolerance = 0.05);
bool finiteAndNormalized(const geometry_msgs::msg::Transform & transform, double tolerance = 0.05);
geometry_msgs::msg::Transform makeTransform(
  double x, double y, double z, double roll, double pitch, double yaw);
geometry_msgs::msg::Transform odomBaseFromRawLidar(
  const geometry_msgs::msg::Pose & world_lidar,
  const geometry_msgs::msg::Transform & base_lidar);
geometry_msgs::msg::Transform mapOdomFromStart(
  double map_x, double map_y, double map_z, double map_yaw,
  const geometry_msgs::msg::Pose & odom_base);

}  // namespace ugv_localization_mvp
