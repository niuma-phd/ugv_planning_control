#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "ugv_gps_waypoint_control/gps_waypoint_core.hpp"
#include "ugv_subject2_mvp/pure_pursuit_controller.hpp"

namespace ugv_gps_waypoint_control
{

class GpsWaypointControllerNode : public rclcpp::Node
{
public:
  GpsWaypointControllerNode()
  : Node("gps_waypoint_controller")
  {
    const auto track_file = declare_parameter<std::string>("track_file", "");
    const auto initial_heading = declare_parameter<std::string>("initial_heading", "");
    motion_enabled_ = declare_parameter<bool>("motion_enabled", false);
    validated_fix_topic_ = declare_parameter<std::string>(
      "validated_fix_topic", "/gps/validated_fix");
    position_valid_topic_ = declare_parameter<std::string>(
      "position_valid_topic", "/gps/gga_position_valid");
    const double course_update_min_distance_m = declare_parameter<double>(
      "course_update_min_distance_m", 5.0);
    fix_timeout_sec_ = declare_parameter<double>("fix_timeout_sec", 2.5);
    maximum_position_speed_mps_ = declare_parameter<double>(
      "maximum_position_speed_mps", 3.0);
    const double watchdog_rate_hz = declare_parameter<double>("watchdog_rate_hz", 20.0);

    ugv_subject2_mvp::ControllerConfig config;
    config.nominal_speed = declare_parameter<double>("nominal_speed", 0.50);
    config.max_speed = declare_parameter<double>("max_speed", 1.00);
    config.max_yaw_rate = declare_parameter<double>("max_yaw_rate", 0.40);
    config.max_curvature = declare_parameter<double>("max_curvature", 1.00);
    config.turn_in_place_threshold_rad = declare_parameter<double>(
      "turn_in_place_threshold_rad", 1.0472);
    config.slowdown_distance = declare_parameter<double>("slowdown_distance", 3.00);
    config.waypoint_tolerance = declare_parameter<double>("waypoint_tolerance", 1.50);
    config.goal_tolerance = declare_parameter<double>("goal_tolerance", 1.50);
    controller_.set_config(config);

    if (validated_fix_topic_.empty() || position_valid_topic_.empty() ||
      !std::isfinite(fix_timeout_sec_) || fix_timeout_sec_ <= 0.0 ||
      !std::isfinite(maximum_position_speed_mps_) || maximum_position_speed_mps_ <= 0.0 ||
      !std::isfinite(watchdog_rate_hz) || watchdog_rate_hz <= 0.0 ||
      !controller_.config_is_valid())
    {
      throw std::invalid_argument(
              "GPS controller topics, timeouts, rates, and motion limits are invalid");
    }

    global_track_ = loadGlobalTrack(track_file);
    projector_ = std::make_unique<Wgs84EnuProjector>(global_track_.front());
    path_.reserve(global_track_.size());
    for (const auto & waypoint : global_track_) {
      const auto local = projector_->project(
        waypoint.longitude_deg, waypoint.latitude_deg, waypoint.altitude_m);
      path_.push_back(ugv_subject2_mvp::Point2D{local.x_east_m, local.y_north_m});
    }
    course_estimator_ = std::make_unique<GpsCourseEstimator>(
      cardinalHeadingToEnuYaw(initial_heading), course_update_min_distance_m);
    if (!course_estimator_->configIsValid()) {
      throw std::invalid_argument("course_update_min_distance_m must be finite and positive");
    }

    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10).reliable());
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/gps_control/target_point", rclcpp::QoS(10).reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/gps_control/status", rclcpp::QoS(1).reliable().transient_local());
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      validated_fix_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&GpsWaypointControllerNode::on_fix, this, std::placeholders::_1));
    position_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      position_valid_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&GpsWaypointControllerNode::on_position_valid, this, std::placeholders::_1));
    watchdog_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / watchdog_rate_hz),
      std::bind(&GpsWaypointControllerNode::on_watchdog, this));

    publish_stop();
    publish_status(motion_enabled_ ? "WAITING_FOR_VALID_GNGGA" : "MOTION_DISABLED");
    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu ordered WGS84 waypoints from '%s'; initial heading=%s; "
      "map axes are x=east, y=north",
      global_track_.size(), track_file.c_str(), initial_heading.c_str());
    if (!motion_enabled_) {
      RCLCPP_WARN(
        get_logger(), "motion_enabled=false; GPS and route status run, but /cmd_vel stays zero");
    }
    RCLCPP_WARN(
      get_logger(),
      "single-antenna GGA has no stationary yaw; moving course updates heading, "
      "and any requested in-place turn is rejected with a latched zero command");
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  static bool finite_fix(const sensor_msgs::msg::NavSatFix & fix)
  {
    return std::isfinite(fix.longitude) && std::isfinite(fix.latitude) &&
           std::isfinite(fix.altitude) && fix.longitude >= -180.0 &&
           fix.longitude <= 180.0 && fix.latitude >= -90.0 && fix.latitude <= 90.0;
  }

  void publish_stop()
  {
    last_command_ = geometry_msgs::msg::Twist{};
    command_pub_->publish(last_command_);
  }

  void publish_status(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
  }

  void reset_gps_history()
  {
    last_fix_received_at_.reset();
    last_position_received_at_.reset();
    last_accepted_position_.reset();
    course_estimator_->resetAnchor();
  }

  void on_position_valid(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (message->data) {
      return;
    }
    reset_gps_history();
    publish_stop();
    publish_status("GPS_QUALITY_INVALID_STOP");
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "GNGGA was rejected or the serial input failed; commanding zero immediately");
  }

  void on_fix(const sensor_msgs::msg::NavSatFix::SharedPtr message)
  {
    const auto received_at = SteadyClock::now();
    if (position_jump_latched_) {
      publish_stop();
      return;
    }
    if (message->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
      !finite_fix(*message))
    {
      reset_gps_history();
      publish_stop();
      publish_status("INVALID_FIX");
      return;
    }

    LocalPoint current;
    try {
      current = projector_->project(message->longitude, message->latitude, message->altitude);
    } catch (const std::exception & error) {
      reset_gps_history();
      publish_stop();
      RCLCPP_ERROR(get_logger(), "cannot project GPS fix: %s", error.what());
      return;
    }
    if (last_accepted_position_ && last_position_received_at_) {
      const double elapsed_sec = std::chrono::duration<double>(
        received_at - *last_position_received_at_).count();
      const double displacement_m = std::hypot(
        current.x_east_m - last_accepted_position_->x_east_m,
        current.y_north_m - last_accepted_position_->y_north_m);
      const double apparent_speed_mps = elapsed_sec > 0.0 ?
        displacement_m / elapsed_sec : std::numeric_limits<double>::infinity();
      if (apparent_speed_mps > maximum_position_speed_mps_) {
        position_jump_latched_ = true;
        last_fix_received_at_.reset();
        publish_stop();
        publish_status("GPS_POSITION_JUMP_STOP_RESTART_REQUIRED");
        RCLCPP_ERROR(
          get_logger(),
          "GPS position changed %.3f m in %.3f s (%.3f m/s), above %.3f m/s; "
          "zero command latched until restart",
          displacement_m, elapsed_sec, apparent_speed_mps, maximum_position_speed_mps_);
        return;
      }
    }
    const bool first_accepted_fix = !last_fix_received_at_.has_value();
    last_fix_received_at_ = received_at;
    last_position_received_at_ = received_at;
    last_accepted_position_ = current;
    CourseObservation course{course_estimator_->yaw(), false};
    // Establish the first anchor immediately. Afterwards, update moving course
    // only while a positive command is actually being sent; stationary
    // single-point drift must not redefine vehicle yaw.
    if (first_accepted_fix || (motion_enabled_ && last_command_.linear.x > 0.05)) {
      course = course_estimator_->observe(current);
    }
    if (course.course_updated) {
      RCLCPP_INFO(
        get_logger(), "[GPS-COURSE] updated moving yaw=%.3f rad from accepted fixes",
        course.yaw_rad);
    }

    ugv_subject2_mvp::ControlInput input;
    input.pose.x = current.x_east_m;
    input.pose.y = current.y_north_m;
    input.pose.yaw = course.yaw_rad;
    input.inputs_valid = true;
    const auto output = controller_.compute(input, path_);
    if (!output.valid) {
      publish_stop();
      publish_status("CONTROL_INPUT_REJECTED");
      return;
    }

    if (output.turning_in_place) {
      heading_unobservable_latched_ = true;
      publish_stop();
      publish_status("HEADING_UNOBSERVABLE_STOP");
      RCLCPP_ERROR(
        get_logger(),
        "controller requested an in-place turn, but GGA cannot observe stationary yaw; "
        "zero command latched until restart");
      return;
    }

    geometry_msgs::msg::Twist command;
    if (motion_enabled_ && !heading_unobservable_latched_ &&
      !output.goal_reached)
    {
      command.linear.x = output.linear_velocity;
      command.angular.z = output.angular_velocity;
    }
    last_command_ = command;
    command_pub_->publish(last_command_);

    geometry_msgs::msg::PointStamped target;
    target.header.stamp = now();
    target.header.frame_id = "gps_enu";
    target.point.x = output.target.x;
    target.point.y = output.target.y;
    target_pub_->publish(target);

    if (!last_target_index_ || output.target_index != *last_target_index_) {
      const std::size_t first_reached = last_target_index_.value_or(0U);
      for (std::size_t index = first_reached; index < output.target_index; ++index) {
        RCLCPP_INFO(
          get_logger(), "[GPS-WAYPOINT] sequence=%ld reached (%zu/%zu)",
          static_cast<long>(global_track_[index].sequence), index + 1U,
          global_track_.size());
      }
      last_target_index_ = output.target_index;
      RCLCPP_INFO(
        get_logger(),
        "[GPS-WAYPOINT] going to sequence=%ld (%zu/%zu), lon=%.8f lat=%.8f",
        static_cast<long>(global_track_[output.target_index].sequence),
        output.target_index + 1U, global_track_.size(),
        global_track_[output.target_index].longitude_deg,
        global_track_[output.target_index].latitude_deg);
    }
    if (output.goal_reached) {
      publish_status("FINAL_WAYPOINT_REACHED");
      if (!goal_reported_) {
        goal_reported_ = true;
        RCLCPP_INFO(
          get_logger(), "[GPS-WAYPOINT] final sequence=%ld reached; zero command latched",
          static_cast<long>(global_track_.back().sequence));
      }
      return;
    }
    publish_status(output.turning_in_place ? "ALIGNING_WITH_LIMITED_YAW" : "TRACKING");
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[GPS-WAYPOINT] sequence=%ld (%zu/%zu) pose_EN=(%.3f, %.3f, yaw=%.3f) "
      "cmd=(v=%.3f, omega=%.3f)",
      static_cast<long>(global_track_[output.target_index].sequence),
      output.target_index + 1U, global_track_.size(), current.x_east_m,
      current.y_north_m, course.yaw_rad, command.linear.x, command.angular.z);
  }

  void on_watchdog()
  {
    if (!motion_enabled_ || heading_unobservable_latched_ || position_jump_latched_) {
      publish_stop();
      return;
    }
    if (!last_fix_received_at_ ||
      std::chrono::duration<double>(SteadyClock::now() - *last_fix_received_at_).count() >
      fix_timeout_sec_)
    {
      reset_gps_history();
      publish_stop();
      publish_status("GPS_FIX_TIMEOUT_STOP");
      return;
    }
    // The receiver is approximately 1 Hz, while many chassis interfaces need
    // a faster command heartbeat. Only repeat a command while its accepted fix
    // remains inside the steady-clock timeout above.
    command_pub_->publish(last_command_);
  }

  bool motion_enabled_{false};
  bool heading_unobservable_latched_{false};
  bool position_jump_latched_{false};
  bool goal_reported_{false};
  double fix_timeout_sec_{2.5};
  double maximum_position_speed_mps_{3.0};
  std::string validated_fix_topic_;
  std::string position_valid_topic_;
  std::vector<GlobalWaypoint> global_track_;
  std::vector<ugv_subject2_mvp::Point2D> path_;
  std::unique_ptr<Wgs84EnuProjector> projector_;
  std::unique_ptr<GpsCourseEstimator> course_estimator_;
  ugv_subject2_mvp::PurePursuitController controller_;
  std::optional<std::size_t> last_target_index_;
  std::optional<SteadyClock::time_point> last_fix_received_at_;
  std::optional<SteadyClock::time_point> last_position_received_at_;
  std::optional<LocalPoint> last_accepted_position_;
  geometry_msgs::msg::Twist last_command_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr position_valid_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace ugv_gps_waypoint_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv_gps_waypoint_control::GpsWaypointControllerNode>());
  rclcpp::shutdown();
  return 0;
}
