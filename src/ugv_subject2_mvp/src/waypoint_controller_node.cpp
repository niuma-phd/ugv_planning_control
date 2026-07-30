#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
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
    config.turn_in_place_threshold_rad = declare_parameter(
      "turn_in_place_threshold_rad", config.turn_in_place_threshold_rad);
    config.slowdown_distance = declare_parameter("slowdown_distance", config.slowdown_distance);
    config.waypoint_tolerance = declare_parameter(
      "waypoint_tolerance", config.waypoint_tolerance);
    config.goal_tolerance = declare_parameter("goal_tolerance", config.goal_tolerance);
    config.enhanced_tracking_enabled = declare_parameter(
      "enhanced_tracking_enabled", config.enhanced_tracking_enabled);
    config.minimum_linear_speed = declare_parameter(
      "minimum_linear_speed", config.minimum_linear_speed);
    config.minimum_tracking_yaw_rate = declare_parameter(
      "minimum_tracking_yaw_rate", config.minimum_tracking_yaw_rate);
    config.minimum_turning_yaw_rate = declare_parameter(
      "minimum_turning_yaw_rate", config.minimum_turning_yaw_rate);
    config.lookahead_min_m = declare_parameter(
      "lookahead_min_m", config.lookahead_min_m);
    config.lookahead_max_m = declare_parameter(
      "lookahead_max_m", config.lookahead_max_m);
    config.lookahead_speed_gain = declare_parameter(
      "lookahead_speed_gain", config.lookahead_speed_gain);
    config.turning_motion_threshold_rad = declare_parameter(
      "turning_motion_threshold_rad", config.turning_motion_threshold_rad);
    config.turn_in_place_exit_threshold_rad = declare_parameter(
      "turn_in_place_exit_threshold_rad", config.turn_in_place_exit_threshold_rad);
    config.tracking_omega_enter_threshold_rad_s = declare_parameter(
      "tracking_omega_enter_threshold_rad_s",
      config.tracking_omega_enter_threshold_rad_s);
    config.tracking_omega_exit_threshold_rad_s = declare_parameter(
      "tracking_omega_exit_threshold_rad_s",
      config.tracking_omega_exit_threshold_rad_s);
    controller_.set_config(config);
    if (!controller_.config_is_valid()) {
      throw std::invalid_argument(
              "motion parameters are invalid; standard Pure Pursuit requires "
              "ordered lookahead, speed-floor, and angular deadband limits");
    }
    waypoint_tolerance_ = config.waypoint_tolerance;

    transform_timeout_sec_ = declare_parameter("transform_timeout_sec", 0.05);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    expected_path_frame_ = declare_parameter<std::string>("expected_path_frame", "map");
    rcl_interfaces::msg::ParameterDescriptor waypoint_file_descriptor;
    waypoint_file_descriptor.description =
      "Absolute CSV path loaded once during node construction";
    waypoint_file_descriptor.read_only = true;
    waypoint_file_ = declare_parameter<std::string>(
      "waypoint_file", "", waypoint_file_descriptor);

    if (expected_path_frame_ != "map") {
      throw std::invalid_argument(
              "expected_path_frame must be 'map' because waypoint CSV coordinates are map-frame");
    }
    if (odom_topic_.empty() || odom_frame_.empty() || base_frame_.empty() ||
      expected_path_frame_.empty() || !std::isfinite(transform_timeout_sec_) ||
      transform_timeout_sec_ < 0.0)
    {
      throw std::invalid_argument(
              "odom topic, frames, and a finite non-negative transform timeout are required");
    }

    const auto loaded_waypoints = load_waypoint_csv(waypoint_file_);
    path_.reserve(loaded_waypoints.size());
    for (const auto & waypoint : loaded_waypoints) {
      path_.push_back(Point2D{waypoint.x_m, waypoint.y_m});
    }

    double shortest_segment = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1U; index < path_.size(); ++index) {
      const double segment_length = std::hypot(
        path_[index].x - path_[index - 1U].x,
        path_[index].y - path_[index - 1U].y);
      if (segment_length > 0.0) {
        shortest_segment = std::min(shortest_segment, segment_length);
      }
    }

    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10).reliable());
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/subject2/target_point", rclcpp::QoS(10).reliable());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&WaypointControllerNode::on_odom, this, std::placeholders::_1));

    publish_stop();
    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu ordered waypoints from '%s'; direct odom input is '%s'",
      path_.size(), waypoint_file_.c_str(), odom_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Motion parameters: nominal_speed=%.3f, max_speed=%.3f, "
      "max_yaw_rate=%.3f, waypoint_tolerance=%.3f, goal_tolerance=%.3f",
      config.nominal_speed, config.max_speed, config.max_yaw_rate,
      config.waypoint_tolerance, config.goal_tolerance);
    RCLCPP_INFO(
      get_logger(),
      "Standard Pure Pursuit=%s | lookahead=[%.3f, %.3f] m + %.3f s * "
      "planned_speed | floors: moving v=%.3f, moving |omega|=%.3f, "
      "in-place breakaway |omega|=%.3f | turning-motion=%.3f rad",
      config.enhanced_tracking_enabled ? "enabled" : "disabled",
      config.lookahead_min_m, config.lookahead_max_m,
      config.lookahead_speed_gain, config.minimum_linear_speed,
      config.minimum_tracking_yaw_rate, config.minimum_turning_yaw_rate,
      config.turning_motion_threshold_rad);
    if (std::isfinite(shortest_segment) && waypoint_tolerance_ >= shortest_segment) {
      RCLCPP_WARN(
        get_logger(),
        "waypoint_tolerance %.3f m is not smaller than the shortest non-zero "
        "CSV segment %.3f m; nearby rows can be confirmed from one pose sample",
        waypoint_tolerance_, shortest_segment);
    }
    RCLCPP_INFO(
      get_logger(),
      "Control is odom-driven: no odom_guard, odom_valid, navigation_enabled, or timestamp gate");
  }

private:
  void publish_stop()
  {
    geometry_msgs::msg::Twist command;
    command_pub_->publish(command);
  }

  bool build_input(
    const nav_msgs::msg::Odometry & odom, ControlInput & input,
    std::string & path_frame)
  {
    path_frame = expected_path_frame_;
    if (odom.header.frame_id != odom_frame_ || odom.child_frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring odom frames '%s' -> '%s'; expected '%s' -> '%s'",
        odom.header.frame_id.c_str(), odom.child_frame_id.c_str(),
        odom_frame_.c_str(), base_frame_.c_str());
      return false;
    }

    geometry_msgs::msg::PoseStamped current_source;
    current_source.header = odom.header;
    // The current pose is consumed as it arrives. Use the latest map->odom TF;
    // no odom timestamp freshness or ordering decision is made here.
    current_source.header.stamp.sec = 0;
    current_source.header.stamp.nanosec = 0;
    current_source.pose = odom.pose.pose;
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
        get_logger(), *get_clock(), 2000,
        "Cannot transform current odom pose into '%s': %s",
        path_frame.c_str(), error.what());
      return false;
    }

    input.pose.x = current_in_path.pose.position.x;
    input.pose.y = current_in_path.pose.position.y;
    input.pose.yaw = tf2::getYaw(current_in_path.pose.orientation);
    input.inputs_valid = true;
    return true;
  }

  void report_status(const ControlInput & input, const ControlOutput & output)
  {
    if (!output.valid) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Controller rejected current pose, route, or motion parameters; commanding zero");
      return;
    }

    if (!last_target_index_ || output.target_index != *last_target_index_) {
      const std::size_t first_confirmed = last_target_index_.value_or(0U);
      for (std::size_t index = first_confirmed; index < output.target_index; ++index) {
        const double waypoint_distance = std::hypot(
          path_[index].x - input.pose.x, path_[index].y - input.pose.y);
        const char * completion = waypoint_distance <= waypoint_tolerance_ ?
          "reached" : "passed";
        RCLCPP_INFO(
          get_logger(), "[WAYPOINT] %s %zu/%zu (x=%.3f, y=%.3f)",
          completion, index + 1U, path_.size(), path_[index].x, path_[index].y);
      }
      last_target_index_ = output.target_index;
      if (!output.goal_reached) {
        RCLCPP_INFO(
          get_logger(), "[WAYPOINT] going to %zu/%zu (x=%.3f, y=%.3f)",
          output.target_index + 1U, path_.size(), output.target.x, output.target.y);
      }
    }

    if (output.goal_reached) {
      if (!goal_reported_) {
        RCLCPP_INFO(
          get_logger(),
          "[WAYPOINT] final waypoint %zu/%zu reached; zero command is latched until restart",
          path_.size(), path_.size());
        goal_reported_ = true;
      }
      return;
    }

    const double target_distance = std::hypot(
      output.target.x - input.pose.x, output.target.y - input.pose.y);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[WAYPOINT] %s %zu/%zu | pose=(%.3f, %.3f, yaw=%.3f) | "
      "distance=%.3f | pursuit=(%.3f, %.3f) seg=%zu Ld=%.3f | "
      "path_yaw=%.3f ref_yaw=%.3f yaw_error=%.3f cross_track=%.3f | "
      "raw=(v=%.3f, omega=%.3f) | "
      "cmd=(v=%.3f, omega=%.3f) floor=(v:%s, omega:%s) breakaway=%s",
      output.turning_in_place ? "aligning to" : "tracking",
      output.target_index + 1U, path_.size(), input.pose.x, input.pose.y,
      input.pose.yaw, target_distance, output.pursuit_target.x,
      output.pursuit_target.y, output.pursuit_segment_index + 1U,
      output.lookahead_distance, output.path_yaw, output.reference_yaw,
      output.yaw_error, output.cross_track_error, output.raw_linear_velocity,
      output.raw_angular_velocity, output.linear_velocity,
      output.angular_velocity, output.minimum_linear_applied ? "yes" : "no",
      output.minimum_angular_applied ? "yes" : "no",
      output.turning_breakaway_active ? "yes" : "no");
  }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    ControlInput input;
    std::string path_frame;
    if (!build_input(*message, input, path_frame)) {
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
    report_status(input, output);

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

  std::vector<Point2D> path_;
  std::optional<std::size_t> last_target_index_;
  bool goal_reported_{false};

  double transform_timeout_sec_{0.05};
  double waypoint_tolerance_{0.30};
  std::string odom_topic_{"/localization/odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string expected_path_frame_{"map"};
  std::string waypoint_file_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
};

}  // namespace ugv_subject2_mvp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv_subject2_mvp::WaypointControllerNode>());
  rclcpp::shutdown();
  return 0;
}
