#include <chrono>
#include <functional>
#include <stdexcept>
#include <cmath>
#include <memory>
#include <limits>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ugv_localization_mvp/ros_time.hpp"
#include "ugv_localization_mvp/transform_math.hpp"

namespace ugv_localization_mvp
{

class MapOdomManagerNode : public rclcpp::Node
{
public:
  MapOdomManagerNode() : Node("map_odom_manager")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    auto_align_ = declare_parameter<bool>("auto_align_from_path", false);
    require_odom_invalid_for_update_ = declare_parameter<bool>(
      "require_odom_invalid_for_update", auto_align_);
    path_segment_epsilon_m_ = declare_parameter<double>("path_segment_epsilon_m", 0.05);
    update_max_age_sec_ = declare_parameter<double>("update_max_age_sec", 1.0);
    update_future_tolerance_sec_ = declare_parameter<double>(
      "update_future_tolerance_sec", 0.1);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);
    if (map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
      map_frame_ == odom_frame_ || map_frame_ == base_frame_ || odom_frame_ == base_frame_ ||
      !std::isfinite(publish_rate_hz) || !std::isfinite(path_segment_epsilon_m_) ||
      !std::isfinite(update_max_age_sec_) || !std::isfinite(update_future_tolerance_sec_) ||
      publish_rate_hz <= 0.0 || path_segment_epsilon_m_ <= 0.0 ||
      update_max_age_sec_ <= 0.0 || update_future_tolerance_sec_ < 0.0)
    {
      throw std::invalid_argument("map/odom/base frames and finite positive rates are required");
    }

    transform_.header.frame_id = map_frame_;
    transform_.child_frame_id = odom_frame_;
    transform_.transform = makeTransform(
      declare_parameter<double>("initial.x", 0.0),
      declare_parameter<double>("initial.y", 0.0),
      declare_parameter<double>("initial.z", 0.0),
      declare_parameter<double>("initial.roll", 0.0),
      declare_parameter<double>("initial.pitch", 0.0),
      declare_parameter<double>("initial.yaw", 0.0));
    if (!finiteAndNormalized(transform_.transform)) {
      throw std::invalid_argument("initial map->odom transform must be finite and normalized");
    }
    transform_ready_ = declare_parameter<bool>("initial_transform_valid", !auto_align_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
      "/localization/map_odom", rclcpp::QoS(1).reliable().transient_local());
    update_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      "/localization/map_odom_update", rclcpp::QoS(10).reliable(),
      std::bind(&MapOdomManagerNode::onUpdate, this, std::placeholders::_1));
    odom_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization/odom_valid", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&MapOdomManagerNode::onOdomValid, this, std::placeholders::_1));
    if (auto_align_) {
      path_sub_ = create_subscription<nav_msgs::msg::Path>(
        declare_parameter<std::string>("path_topic", "/subject2/path"), rclcpp::QoS(1).reliable(),
        std::bind(&MapOdomManagerNode::onPath, this, std::placeholders::_1));
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter<std::string>("odom_topic", "/localization/trusted_odom"), rclcpp::QoS(10),
        std::bind(&MapOdomManagerNode::onOdom, this, std::placeholders::_1));
      RCLCPP_WARN(get_logger(), "waiting to latch map->odom from path start and first canonical odom");
    } else if (!transform_ready_) {
      RCLCPP_WARN(get_logger(), "waiting for an explicit map->odom update");
    }
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      std::bind(&MapOdomManagerNode::publish, this));
  }

private:
  struct PathStart {double x; double y; double z; double yaw;};

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto_alignment_latched_ || path_start_ || msg->header.frame_id != map_frame_ || msg->poses.size() < 2U) {return;}
    const auto & first = msg->poses.front().pose.position;
    if (!std::isfinite(first.x) || !std::isfinite(first.y) || !std::isfinite(first.z)) {return;}
    for (std::size_t i = 1; i < msg->poses.size(); ++i) {
      const auto & next = msg->poses[i].pose.position;
      const double dx = next.x - first.x;
      const double dy = next.y - first.y;
      if (!std::isfinite(next.x) || !std::isfinite(next.y) || !std::isfinite(next.z)) {return;}
      if (std::hypot(dx, dy) >= path_segment_epsilon_m_) {
        path_start_ = PathStart{first.x, first.y, first.z, std::atan2(dy, dx)};
        tryAutoAlign();
        return;
      }
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "path has no non-coincident segment for start heading");
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto_alignment_latched_ || msg->header.frame_id != odom_frame_ ||
      msg->child_frame_id != base_frame_ ||
      !positiveRosTimeToNanoseconds(msg->header.stamp) ||
      !finiteAndNormalized(msg->pose.pose)) {return;}
    if (!first_odom_pose_) {first_odom_pose_ = msg->pose.pose;}
    tryAutoAlign();
  }

  void tryAutoAlign()
  {
    if (!path_start_ || !first_odom_pose_ || auto_alignment_latched_) {return;}
    transform_.transform = mapOdomFromStart(
      path_start_->x, path_start_->y, path_start_->z, path_start_->yaw, *first_odom_pose_);
    transform_ready_ = true;
    auto_alignment_latched_ = true;
    RCLCPP_INFO(
      get_logger(), "latched map->odom from path start (%.3f, %.3f, yaw %.3f)",
      path_start_->x, path_start_->y, path_start_->yaw);
  }

  void onUpdate(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    const auto stamp_ns = positiveRosTimeToNanoseconds(msg->header.stamp);
    const double age_sec = stamp_ns ?
      static_cast<double>(now().nanoseconds() - *stamp_ns) / 1.0e9 :
      std::numeric_limits<double>::infinity();
    if (msg->header.frame_id != map_frame_ || msg->child_frame_id != odom_frame_ ||
      !stamp_ns || age_sec > update_max_age_sec_ || age_sec < -update_future_tolerance_sec_ ||
      !finiteAndNormalized(msg->transform))
    {
      RCLCPP_ERROR(get_logger(), "rejected invalid or frame-mismatched map_odom_update");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (require_odom_invalid_for_update_ && (!odom_valid_received_ || odom_valid_)) {
      RCLCPP_ERROR(
        get_logger(),
        "rejected map_odom_update; an explicit odom_valid=false gate is required");
      return;
    }
    transform_.transform = msg->transform;
    transform_.transform.rotation = normalizedQuaternion(msg->transform.rotation);
    transform_ready_ = true;
    auto_alignment_latched_ = true;
    RCLCPP_INFO(get_logger(), "accepted explicit map->odom update");
  }

  void onOdomValid(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_valid_ = msg->data;
    odom_valid_received_ = true;
  }

  void publish()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!transform_ready_) {return;}
    transform_.header.stamp = now();
    tf_broadcaster_->sendTransform(transform_);
    transform_pub_->publish(transform_);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool auto_align_{false};
  bool require_odom_invalid_for_update_{false};
  bool transform_ready_{false};
  bool auto_alignment_latched_{false};
  bool odom_valid_{false};
  bool odom_valid_received_{false};
  double path_segment_epsilon_m_{0.05};
  double update_max_age_sec_{1.0};
  double update_future_tolerance_sec_{0.1};
  std::mutex mutex_;
  geometry_msgs::msg::TransformStamped transform_;
  std::optional<PathStart> path_start_;
  std::optional<geometry_msgs::msg::Pose> first_odom_pose_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr update_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odom_valid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ugv_localization_mvp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv_localization_mvp::MapOdomManagerNode>());
  rclcpp::shutdown();
  return 0;
}
