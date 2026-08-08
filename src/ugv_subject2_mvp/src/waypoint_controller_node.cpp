#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
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
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "ugv_subject2_mvp/pure_pursuit_controller.hpp"
#include "ugv_subject2_mvp/stall_boost_controller.hpp"
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
    maximum_yaw_rate_ = config.max_yaw_rate;
    minimum_tracking_yaw_rate_ = config.minimum_tracking_yaw_rate;

    command_publish_rate_hz_ = declare_parameter("command_publish_rate_hz", 20.0);
    command_hold_timeout_sec_ = declare_parameter("command_hold_timeout_sec", 0.30);
    tracking_yaw_pulse_enabled_ = declare_parameter(
      "tracking_yaw_pulse_enabled", true);
    tracking_yaw_min_pulse_sec_ = declare_parameter(
      "tracking_yaw_min_pulse_sec", 0.10);
    tracking_yaw_demand_gain_ = declare_parameter(
      "tracking_yaw_demand_gain", 1.0);
    if (!std::isfinite(command_publish_rate_hz_) ||
      command_publish_rate_hz_ <= 0.0 ||
      !std::isfinite(command_hold_timeout_sec_) ||
      command_hold_timeout_sec_ <= 2.0 / command_publish_rate_hz_ ||
      !std::isfinite(tracking_yaw_min_pulse_sec_) ||
      tracking_yaw_min_pulse_sec_ <= 0.0 ||
      !std::isfinite(tracking_yaw_demand_gain_) ||
      tracking_yaw_demand_gain_ <= 0.0)
    {
      throw std::invalid_argument(
              "fixed command publishing and tracking-yaw pulse parameters are invalid");
    }
    tracking_yaw_shaper_.set_config(
      command_publish_rate_hz_, minimum_tracking_yaw_rate_,
      tracking_yaw_min_pulse_sec_);
    if (!tracking_yaw_shaper_.config_is_valid()) {
      throw std::invalid_argument("tracking-yaw pulse shaper configuration is invalid");
    }

    StallBoostConfig stall_boost_config;
    stall_boost_config.enabled = declare_parameter(
      "stall_boost_enabled", stall_boost_config.enabled);
    stall_boost_config.detection_duration_sec = declare_parameter(
      "stall_detection_duration_sec", stall_boost_config.detection_duration_sec);
    stall_boost_config.maximum_observation_gap_sec = declare_parameter(
      "stall_observation_gap_reset_sec",
      stall_boost_config.maximum_observation_gap_sec);
    stall_boost_config.motion_translation_threshold_m = declare_parameter(
      "stall_motion_translation_threshold_m",
      stall_boost_config.motion_translation_threshold_m);
    stall_boost_config.motion_yaw_threshold_rad = declare_parameter(
      "stall_motion_yaw_threshold_rad", stall_boost_config.motion_yaw_threshold_rad);
    stall_boost_config.minimum_linear_command_mps = declare_parameter(
      "stall_min_linear_command_mps", stall_boost_config.minimum_linear_command_mps);
    stall_boost_config.minimum_angular_command_rad_s = declare_parameter(
      "stall_min_angular_command_rad_s",
      stall_boost_config.minimum_angular_command_rad_s);
    stall_boost_config.linear_boost_speed_mps = declare_parameter(
      "stall_linear_boost_speed_mps", stall_boost_config.linear_boost_speed_mps);
    stall_boost_config.angular_boost_rate_rad_s = declare_parameter(
      "stall_angular_boost_rate_rad_s",
      stall_boost_config.angular_boost_rate_rad_s);
    stall_boost_config.boost_min_duration_sec = declare_parameter(
      "stall_boost_min_duration_sec", stall_boost_config.boost_min_duration_sec);
    stall_boost_config.boost_max_duration_sec = declare_parameter(
      "stall_boost_max_duration_sec", stall_boost_config.boost_max_duration_sec);
    stall_boost_config.ramp_down_sec = declare_parameter(
      "stall_boost_ramp_down_sec", stall_boost_config.ramp_down_sec);
    stall_boost_config.cooldown_sec = declare_parameter(
      "stall_boost_cooldown_sec", stall_boost_config.cooldown_sec);
    const auto stall_boost_max_attempts = declare_parameter<std::int64_t>(
      "stall_boost_max_attempts", stall_boost_config.max_attempts);
    if (stall_boost_max_attempts < 1 ||
      stall_boost_max_attempts > std::numeric_limits<int>::max())
    {
      throw std::invalid_argument("stall_boost_max_attempts is outside integer range");
    }
    stall_boost_config.max_attempts = static_cast<int>(stall_boost_max_attempts);
    stall_boost_.set_config(stall_boost_config);
    const bool enabled_boost_exceeds_limits = stall_boost_config.enabled &&
      (stall_boost_config.linear_boost_speed_mps > config.max_speed ||
      stall_boost_config.angular_boost_rate_rad_s > config.max_yaw_rate);
    const bool enabled_observation_gap_exceeds_lease =
      stall_boost_config.enabled &&
      stall_boost_config.maximum_observation_gap_sec > command_hold_timeout_sec_;
    if (!stall_boost_.config_is_valid() || enabled_boost_exceeds_limits ||
      enabled_observation_gap_exceeds_lease)
    {
      throw std::invalid_argument(
              "stall boost parameters are invalid, exceed controller limits, "
              "or exceed the command lease");
    }
    stall_boost_config_ = stall_boost_config;

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

    odom_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    command_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = odom_callback_group_;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&WaypointControllerNode::on_odom, this, std::placeholders::_1),
      odom_options);
    const auto command_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / command_publish_rate_hz_));
    command_timer_ = create_wall_timer(
      command_period,
      std::bind(&WaypointControllerNode::publish_cached_command, this),
      command_callback_group_);

    cache_stop_and_publish();
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
      "in-place |omega|=%.3f | turning-motion-confirmed=%.3f rad",
      config.enhanced_tracking_enabled ? "enabled" : "disabled",
      config.lookahead_min_m, config.lookahead_max_m,
      config.lookahead_speed_gain, config.minimum_linear_speed,
      config.minimum_tracking_yaw_rate, config.minimum_turning_yaw_rate,
      config.turning_motion_threshold_rad);
    RCLCPP_INFO(
      get_logger(),
      "Actuation: fixed %.1f Hz /cmd_vel, %.3f s receive lease; "
      "TRACK sub-floor yaw pulse=%s, amplitude=%.3f rad/s, "
      "minimum pulse=%.3f s, demand gain=%.3f",
      command_publish_rate_hz_, command_hold_timeout_sec_,
      tracking_yaw_pulse_enabled_ ? "enabled" : "disabled",
      minimum_tracking_yaw_rate_, tracking_yaw_min_pulse_sec_,
      tracking_yaw_demand_gain_);
    if (std::isfinite(shortest_segment) && waypoint_tolerance_ >= shortest_segment) {
      RCLCPP_WARN(
        get_logger(),
        "waypoint_tolerance %.3f m is not smaller than the shortest non-zero "
        "CSV segment %.3f m; nearby rows can be confirmed from one pose sample",
        waypoint_tolerance_, shortest_segment);
    }
    RCLCPP_INFO(
      get_logger(),
      "Stall boost=%s | detect=%.2f s gap-reset=%.2f s "
      "motion=(%.3f m, %.3f rad) | "
      "TRACK |v|->%.2f m/s, ALIGN |omega|->%.2f rad/s | "
      "boost=[%.2f, %.2f] s ramp=%.2f s cooldown=%.2f s attempts=%d",
      stall_boost_config_.enabled ? "enabled" : "disabled",
      stall_boost_config_.detection_duration_sec,
      stall_boost_config_.maximum_observation_gap_sec,
      stall_boost_config_.motion_translation_threshold_m,
      stall_boost_config_.motion_yaw_threshold_rad,
      stall_boost_config_.linear_boost_speed_mps,
      stall_boost_config_.angular_boost_rate_rad_s,
      stall_boost_config_.boost_min_duration_sec,
      stall_boost_config_.boost_max_duration_sec,
      stall_boost_config_.ramp_down_sec, stall_boost_config_.cooldown_sec,
      stall_boost_config_.max_attempts);
    RCLCPP_INFO(
      get_logger(),
      "Control is odom-driven: no odom_guard, odom_valid, navigation_enabled, or timestamp gate");
  }

private:
  struct CachedCommand
  {
    geometry_msgs::msg::Twist command;
    std::chrono::steady_clock::time_point received_at{};
    std::uint64_t generation{0U};
    double tracking_yaw_demand{0.0};
    bool initialized{false};
    bool tracking_yaw_pulse{false};
  };

  void publish_stop()
  {
    geometry_msgs::msg::Twist command;
    command_pub_->publish(command);
  }

  void cache_stop_and_publish()
  {
    stall_boost_.reset();
    std::lock_guard<std::mutex> publication_lock(publication_mutex_);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      cached_command_.command = geometry_msgs::msg::Twist{};
      cached_command_.received_at = std::chrono::steady_clock::now();
      ++cached_command_.generation;
      cached_command_.tracking_yaw_demand = 0.0;
      cached_command_.initialized = true;
      cached_command_.tracking_yaw_pulse = false;
    }
    tracking_yaw_shaper_.reset();
    publish_stop();
  }

  void cache_controller_output(
    const ControlOutput & output, const StallBoostOutput & stall_output)
  {
    CachedCommand next;
    next.received_at = std::chrono::steady_clock::now();
    next.initialized = true;
    if (output.valid && !output.goal_reached) {
      next.command.linear.x = stall_output.linear_velocity;
      const bool use_tracking_yaw_pulse =
        tracking_yaw_pulse_enabled_ &&
        !output.turning_in_place &&
        output.yaw_correction_active &&
        output.minimum_angular_applied;
      if (use_tracking_yaw_pulse) {
        const double amplified_demand = std::clamp(
          std::abs(output.raw_angular_velocity) * tracking_yaw_demand_gain_,
          0.0, maximum_yaw_rate_);
        next.tracking_yaw_demand = std::copysign(
          amplified_demand, output.angular_velocity);
        next.tracking_yaw_pulse = true;
      } else {
        next.command.angular.z = stall_output.angular_velocity;
      }
    }

    std::lock_guard<std::mutex> publication_lock(publication_mutex_);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      next.generation = cached_command_.generation + 1U;
      cached_command_ = next;
    }
  }

  void publish_cached_command()
  {
    std::lock_guard<std::mutex> publication_lock(publication_mutex_);
    CachedCommand state;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      state = cached_command_;
    }

    geometry_msgs::msg::Twist command;
    bool lease_expired = false;
    if (state.initialized) {
      const double command_age_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - state.received_at).count();
      lease_expired = command_age_sec > command_hold_timeout_sec_;
      if (!lease_expired) {
        command = state.command;
      }
    }

    if (!lease_expired && state.initialized && state.tracking_yaw_pulse) {
      command.angular.z = tracking_yaw_shaper_.step(
        state.tracking_yaw_demand, true);
    } else {
      tracking_yaw_shaper_.reset();
    }

    if (lease_expired &&
      (state.command.linear.x != 0.0 || state.command.angular.z != 0.0 ||
      state.tracking_yaw_pulse))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No newly received usable odom/control result for %.3f s; "
        "fixed-rate publisher is commanding zero",
        command_hold_timeout_sec_);
    }
    command_pub_->publish(command);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[ACTUATION] generation=%llu pulse=%s demand_omega=%.3f "
      "published=(v=%.3f, omega=%.3f)",
      static_cast<unsigned long long>(state.generation),
      state.tracking_yaw_pulse ? "yes" : "no",
      state.tracking_yaw_demand, command.linear.x, command.angular.z);
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
      "controller=(v=%.3f, omega=%.3f) floor=(v:%s, omega:%s) breakaway=%s",
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

  void report_stall_boost(const StallBoostOutput & output)
  {
    if (output.phase_changed) {
      RCLCPP_WARN(
        get_logger(),
        "[STALL_BOOST] %s %s -> %s attempt=%d motion=(%.3f m, %.3f rad) "
        "command=(v=%.3f, omega=%.3f)",
        stall_boost_mode_name(output.mode),
        stall_boost_phase_name(output.previous_phase),
        stall_boost_phase_name(output.phase), output.attempt_count,
        output.translation_excursion_m, output.yaw_excursion_rad,
        output.linear_velocity, output.angular_velocity);
    }
    if (output.phase == StallBoostPhase::boosting ||
      output.phase == StallBoostPhase::ramp_down ||
      output.phase == StallBoostPhase::suppressed)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[STALL_BOOST] mode=%s phase=%s attempt=%d motion=(%.3f m, %.3f rad) "
        "shaped=(v=%.3f, omega=%.3f)",
        stall_boost_mode_name(output.mode), stall_boost_phase_name(output.phase),
        output.attempt_count, output.translation_excursion_m,
        output.yaw_excursion_rad, output.linear_velocity, output.angular_velocity);
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    ControlInput input;
    std::string path_frame;
    if (!build_input(*message, input, path_frame)) {
      cache_stop_and_publish();
      return;
    }

    const ControlOutput output = controller_.compute(input, path_);
    StallBoostInput stall_input;
    stall_input.steady_time_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    stall_input.pose = input.pose;
    stall_input.desired_linear_velocity = output.linear_velocity;
    stall_input.desired_angular_velocity = output.angular_velocity;
    stall_input.target_index = output.target_index;
    stall_input.command_valid = output.valid && !output.goal_reached;
    stall_input.turning_in_place = output.turning_in_place;
    const StallBoostOutput stall_output = stall_boost_.step(stall_input);

    if (!output.valid || output.goal_reached) {
      cache_stop_and_publish();
    } else {
      cache_controller_output(output, stall_output);
    }
    report_status(input, output);
    report_stall_boost(stall_output);

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
  StallBoostController stall_boost_;
  StallBoostConfig stall_boost_config_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<Point2D> path_;
  std::optional<std::size_t> last_target_index_;
  bool goal_reported_{false};

  double transform_timeout_sec_{0.05};
  double waypoint_tolerance_{0.30};
  double maximum_yaw_rate_{0.8};
  double minimum_tracking_yaw_rate_{1.0};
  double command_publish_rate_hz_{20.0};
  double command_hold_timeout_sec_{0.30};
  double tracking_yaw_min_pulse_sec_{0.10};
  double tracking_yaw_demand_gain_{1.0};
  bool tracking_yaw_pulse_enabled_{true};
  std::string odom_topic_{"/localization/odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string expected_path_frame_{"map"};
  std::string waypoint_file_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;

  std::mutex command_mutex_;
  std::mutex publication_mutex_;
  CachedCommand cached_command_;
  TrackingYawPulseShaper tracking_yaw_shaper_;
};

}  // namespace ugv_subject2_mvp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ugv_subject2_mvp::WaypointControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
