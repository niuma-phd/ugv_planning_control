#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "ugv_subject2_mvp/pure_pursuit_controller.hpp"

namespace ugv_subject2_mvp
{

class WaypointControllerNode : public rclcpp::Node
{
public:
  WaypointControllerNode()
  : Node("waypoint_controller_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    ControllerConfig config;
    config.nominal_speed = declare_parameter("nominal_speed", config.nominal_speed);
    config.max_speed = declare_parameter("max_speed", config.max_speed);
    config.max_yaw_rate = declare_parameter("max_yaw_rate", config.max_yaw_rate);
    config.max_curvature = declare_parameter("max_curvature", config.max_curvature);
    config.lookahead_distance = declare_parameter("lookahead_distance", config.lookahead_distance);
    config.use_speed_scaled_lookahead = declare_parameter(
      "use_speed_scaled_lookahead", config.use_speed_scaled_lookahead);
    config.lookahead_speed_gain = declare_parameter(
      "lookahead_speed_gain", config.lookahead_speed_gain);
    config.min_lookahead = declare_parameter("min_lookahead", config.min_lookahead);
    config.max_lookahead = declare_parameter("max_lookahead", config.max_lookahead);
    config.slowdown_distance = declare_parameter("slowdown_distance", config.slowdown_distance);
    config.goal_tolerance = declare_parameter("goal_tolerance", config.goal_tolerance);
    const auto progress_search_ahead = declare_parameter<int>("progress_search_ahead", 200);
    const auto progress_backtrack = declare_parameter<int>("progress_backtrack", 3);
    config.progress_search_ahead = static_cast<std::size_t>(
      std::max<std::int64_t>(0, progress_search_ahead));
    config.progress_backtrack = static_cast<std::size_t>(
      std::max<std::int64_t>(0, progress_backtrack));
    controller_.set_config(config);

    control_rate_hz_ = declare_parameter("control_rate_hz", 20.0);
    odom_timeout_sec_ = declare_parameter("odom_timeout_sec", 0.3);
    path_timeout_sec_ = declare_parameter("path_timeout_sec", 5.0);
    valid_timeout_sec_ = declare_parameter("valid_timeout_sec", 0.5);
    transform_timeout_sec_ = declare_parameter("transform_timeout_sec", 0.05);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    if (!std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0) {
      throw std::invalid_argument("control_rate_hz must be positive");
    }
    if (base_frame_.empty() || !std::isfinite(odom_timeout_sec_) ||
      !std::isfinite(path_timeout_sec_) || !std::isfinite(valid_timeout_sec_) ||
      !std::isfinite(transform_timeout_sec_) || odom_timeout_sec_ <= 0.0 ||
      path_timeout_sec_ <= 0.0 || valid_timeout_sec_ <= 0.0 ||
      transform_timeout_sec_ < 0.0)
    {
      throw std::invalid_argument("base_frame and finite positive input timeouts are required");
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/trusted_odom", rclcpp::QoS(10).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        odom_ = std::move(message);
        odom_received_at_ = std::chrono::steady_clock::now();
      });
    valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization/odom_valid", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        odom_valid_ = message->data;
        if (!odom_valid_) {
          odom_.reset();
          controller_.reset_progress();
        }
        valid_received_ = true;
        valid_received_at_ = std::chrono::steady_clock::now();
      });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/subject2/path", rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {
        const bool geometry_changed = !path_ || !same_path_geometry(*path_, *message);
        path_ = std::move(message);
        path_received_at_ = std::chrono::steady_clock::now();
        if (geometry_changed) {
          controller_.reset_progress();
        }
      });

    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10).reliable());
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/subject2/target_point", rclcpp::QoS(10).reliable());

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WaypointControllerNode::control_tick, this));
  }

private:
  static bool same_path_geometry(
    const nav_msgs::msg::Path & first, const nav_msgs::msg::Path & second)
  {
    if (first.header.frame_id != second.header.frame_id ||
      first.poses.size() != second.poses.size())
    {
      return false;
    }
    for (std::size_t index = 0U; index < first.poses.size(); ++index) {
      const auto & first_pose = first.poses[index];
      const auto & second_pose = second.poses[index];
      if (first_pose.header.frame_id != second_pose.header.frame_id ||
        first_pose.pose.position.x != second_pose.pose.position.x ||
        first_pose.pose.position.y != second_pose.pose.position.y)
      {
        return false;
      }
    }
    return true;
  }

  static bool recent(
    const std::chrono::steady_clock::time_point & received_at, const double timeout)
  {
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - received_at).count();
    return timeout > 0.0 && age <= timeout;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist command;
    command_pub_->publish(command);
  }

  bool build_input(ControlInput & input, std::string & path_frame)
  {
    if (!odom_ || !path_ || path_->poses.empty() || !valid_received_ || !odom_valid_ ||
      !recent(odom_received_at_, odom_timeout_sec_) ||
      !recent(path_received_at_, path_timeout_sec_) ||
      !recent(valid_received_at_, valid_timeout_sec_))
    {
      return false;
    }

    path_frame = path_->header.frame_id;
    if (path_frame.empty() || odom_->header.frame_id != "odom" ||
      odom_->child_frame_id != base_frame_)
    {
      return false;
    }

    geometry_msgs::msg::PoseStamped current_source;
    current_source.header = odom_->header;
    // Freshness is checked above with a monotonic receive timestamp. Use the
    // latest map->odom transform here because that transform is broadcast by a
    // separate timer and is not guaranteed to bracket the odometry stamp.
    current_source.header.stamp.sec = 0;
    current_source.header.stamp.nanosec = 0;
    current_source.pose = odom_->pose.pose;
    geometry_msgs::msg::PoseStamped current_in_path;
    try {
      if (current_source.header.frame_id == path_frame) {
        current_in_path = current_source;
      } else {
        current_in_path = tf_buffer_.transform(
          current_source, path_frame, tf2::durationFromSec(transform_timeout_sec_));
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot transform odom pose into path frame: %s",
        error.what());
      return false;
    }

    input.pose.x = current_in_path.pose.position.x;
    input.pose.y = current_in_path.pose.position.y;
    input.pose.yaw = tf2::getYaw(current_in_path.pose.orientation);
    input.current_speed = odom_->twist.twist.linear.x;
    input.path.reserve(path_->poses.size());

    for (const auto & pose : path_->poses) {
      const std::string source_frame = pose.header.frame_id.empty() ? path_frame : pose.header.frame_id;
      if (source_frame == path_frame) {
        input.path.push_back(Point2D{pose.pose.position.x, pose.pose.position.y});
        continue;
      }

      geometry_msgs::msg::PointStamped source;
      source.header = pose.header;
      source.header.frame_id = source_frame;
      source.point = pose.pose.position;
      try {
        const auto transformed = tf_buffer_.transform(
          source, path_frame, tf2::durationFromSec(transform_timeout_sec_));
        input.path.push_back(Point2D{transformed.point.x, transformed.point.y});
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Cannot transform path point: %s", error.what());
        return false;
      }
    }
    input.inputs_valid = true;
    return true;
  }

  void control_tick()
  {
    ControlInput input;
    std::string path_frame;
    if (!build_input(input, path_frame)) {
      publish_stop();
      return;
    }

    const ControlOutput output = controller_.compute(input);
    geometry_msgs::msg::Twist command;
    if (output.valid && !output.goal_reached) {
      command.linear.x = output.linear_velocity;
      command.angular.z = output.angular_velocity;
    }
    command_pub_->publish(command);

    if (output.valid) {
      geometry_msgs::msg::PointStamped target;
      target.header.stamp = now();
      target.header.frame_id = path_frame;
      target.point.x = output.target.x;
      target.point.y = output.target.y;
      target_pub_->publish(target);
    }
  }

  PurePursuitController controller_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  nav_msgs::msg::Path::ConstSharedPtr path_;
  bool odom_valid_{false};
  bool valid_received_{false};
  std::chrono::steady_clock::time_point odom_received_at_{};
  std::chrono::steady_clock::time_point path_received_at_{};
  std::chrono::steady_clock::time_point valid_received_at_{};

  double control_rate_hz_{20.0};
  double odom_timeout_sec_{0.3};
  double path_timeout_sec_{5.0};
  double valid_timeout_sec_{0.5};
  double transform_timeout_sec_{0.05};
  std::string base_frame_{"base_link"};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ugv_subject2_mvp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv_subject2_mvp::WaypointControllerNode>());
  rclcpp::shutdown();
  return 0;
}
