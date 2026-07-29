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
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "ugv_subject2_mvp/pure_pursuit_controller.hpp"
#include "ugv_subject2_mvp/waypoint_file_loader.hpp"

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
    valid_timeout_sec_ = declare_parameter("valid_timeout_sec", 0.5);
    enable_timeout_sec_ = declare_parameter("enable_timeout_sec", 0.5);
    transform_timeout_sec_ = declare_parameter("transform_timeout_sec", 0.05);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    expected_path_frame_ = declare_parameter<std::string>("expected_path_frame", "map");
    rcl_interfaces::msg::ParameterDescriptor waypoint_file_descriptor;
    waypoint_file_descriptor.description =
      "Absolute CSV path loaded once during node construction";
    waypoint_file_descriptor.read_only = true;
    waypoint_file_ = declare_parameter<std::string>(
      "waypoint_file", "", waypoint_file_descriptor);

    if (!std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0) {
      throw std::invalid_argument("control_rate_hz must be positive");
    }
    if (expected_path_frame_ != "map") {
      throw std::invalid_argument(
              "expected_path_frame must be 'map' because waypoint CSV coordinates are map-frame");
    }
    if (base_frame_.empty() || expected_path_frame_.empty() ||
      !std::isfinite(odom_timeout_sec_) ||
      !std::isfinite(valid_timeout_sec_) ||
      !std::isfinite(enable_timeout_sec_) ||
      !std::isfinite(transform_timeout_sec_) || odom_timeout_sec_ <= 0.0 ||
      valid_timeout_sec_ <= 0.0 ||
      enable_timeout_sec_ <= 0.0 ||
      transform_timeout_sec_ < 0.0)
    {
      throw std::invalid_argument(
              "base/path frames and finite positive input timeouts are required");
    }

    // The route is immutable for the lifetime of this node. Loading it before
    // subscriptions, publishers, and the control timer prevents a partially
    // constructed controller from producing output after configuration errors.
    const auto loaded_waypoints = load_waypoint_csv(waypoint_file_);
    path_.reserve(loaded_waypoints.size());
    for (const auto & waypoint : loaded_waypoints) {
      path_.push_back(Point2D{waypoint.x_m, waypoint.y_m});
    }
    RCLCPP_INFO(
      get_logger(), "Loaded %zu fixed waypoints from '%s' in frame '%s'",
      path_.size(), waypoint_file_.c_str(), expected_path_frame_.c_str());

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
          publish_stop();
        }
        valid_received_ = true;
        valid_received_at_ = std::chrono::steady_clock::now();
      });
    navigation_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization/navigation_enabled", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        navigation_enabled_ = message->data;
        navigation_enable_received_ = true;
        navigation_enable_received_at_ = std::chrono::steady_clock::now();
        if (!navigation_enabled_) {
          controller_.reset_progress();
          publish_stop();
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
    if (!odom_ || path_.empty() || !valid_received_ || !odom_valid_ ||
      !navigation_enable_received_ || !navigation_enabled_ ||
      !recent(odom_received_at_, odom_timeout_sec_) ||
      !recent(valid_received_at_, valid_timeout_sec_) ||
      !recent(navigation_enable_received_at_, enable_timeout_sec_))
    {
      return false;
    }

    path_frame = expected_path_frame_;
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

    const ControlOutput output = controller_.compute(input, path_);
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
  std::vector<Point2D> path_;
  bool odom_valid_{false};
  bool valid_received_{false};
  bool navigation_enabled_{false};
  bool navigation_enable_received_{false};
  std::chrono::steady_clock::time_point odom_received_at_{};
  std::chrono::steady_clock::time_point valid_received_at_{};
  std::chrono::steady_clock::time_point navigation_enable_received_at_{};

  double control_rate_hz_{20.0};
  double odom_timeout_sec_{0.3};
  double valid_timeout_sec_{0.5};
  double enable_timeout_sec_{0.5};
  double transform_timeout_sec_{0.05};
  std::string base_frame_{"base_link"};
  std::string expected_path_frame_{"map"};
  std::string waypoint_file_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_enable_sub_;
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
