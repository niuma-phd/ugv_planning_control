#include "ugv_subject1_avoidance_mvp/local_avoidance.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace ugv_subject1_avoidance_mvp
{

class LocalAvoidanceNode : public rclcpp::Node
{
public:
  LocalAvoidanceNode()
  : Node("local_avoidance_node")
  {
    PlannerConfig config;
    config.speed_mps = declare_parameter("speed_mps", config.speed_mps);
    config.max_curvature = declare_parameter("max_curvature", config.max_curvature);
    config.curvature_samples = declare_parameter("curvature_samples", config.curvature_samples);
    config.horizon_m = declare_parameter("horizon_m", config.horizon_m);
    config.step_m = declare_parameter("step_m", config.step_m);
    config.footprint_half_length_m = declare_parameter(
      "footprint_half_length_m", config.footprint_half_length_m);
    config.footprint_half_width_m = declare_parameter(
      "footprint_half_width_m", config.footprint_half_width_m);
    config.inflation_m = declare_parameter("inflation_m", config.inflation_m);
    config.goal_distance_weight = declare_parameter(
      "goal_distance_weight", config.goal_distance_weight);
    config.heading_weight = declare_parameter("heading_weight", config.heading_weight);
    config.curvature_weight = declare_parameter("curvature_weight", config.curvature_weight);
    config.clearance_weight = declare_parameter("clearance_weight", config.clearance_weight);
    config.max_clearance_reward_m = declare_parameter(
      "max_clearance_reward_m", config.max_clearance_reward_m);
    input_timeout_ = std::chrono::duration<double>(
      declare_parameter("input_timeout_s", 0.30));
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    planner_ = std::make_unique<LocalAvoidance>(config);
    const auto qos = rclcpp::SensorDataQoS();
    obstacles_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/subject1/obstacles", qos,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr message) {
        if (!frame_id_matches(message->header.frame_id, frame_id_)) {
          have_obstacles_ = false;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "ignoring obstacles in frame '%s'; expected '%s'",
            message->header.frame_id.c_str(), frame_id_.c_str());
          return;
        }
        obstacles_.clear();
        obstacles_.reserve(message->poses.size());
        for (const auto & pose : message->poses) {
          obstacles_.push_back({pose.position.x, pose.position.y});
        }
        obstacle_receive_time_ = std::chrono::steady_clock::now();
        have_obstacles_ = true;
      });
    waypoint_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/subject1/next_waypoint_base", qos,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr message) {
        if (!frame_id_matches(message->header.frame_id, frame_id_)) {
          have_waypoint_ = false;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "ignoring waypoint in frame '%s'; expected '%s'",
            message->header.frame_id.c_str(), frame_id_.c_str());
          return;
        }
        goal_ = {message->point.x, message->point.y};
        waypoint_receive_time_ = std::chrono::steady_clock::now();
        have_waypoint_ = true;
      });

    active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/subject1/avoidance_active", rclcpp::QoS(1).reliable());
    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/subject1/avoid_cmd_vel", rclcpp::QoS(1).reliable());
    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/subject1/selected_trajectory", rclcpp::QoS(1).reliable());

    if (publish_rate_hz_ <= 0.0 || input_timeout_.count() <= 0.0) {
      throw std::invalid_argument("publish_rate_hz and input_timeout_s must be positive");
    }
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      [this]() {publish();});
  }

private:
  void publish()
  {
    const auto now_steady = std::chrono::steady_clock::now();
    const bool fresh = have_obstacles_ && have_waypoint_ &&
      now_steady - obstacle_receive_time_ <= input_timeout_ &&
      now_steady - waypoint_receive_time_ <= input_timeout_;
    const PlanResult result = planner_->plan(obstacles_, goal_, fresh);
    const auto stamp = now();

    std_msgs::msg::Bool active;
    active.data = result.active;
    active_pub_->publish(active);

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = stamp;
    command.header.frame_id = frame_id_;
    command.twist.linear.x = result.speed_mps;
    command.twist.angular.z = result.yaw_rate_radps;
    command_pub_->publish(command);

    nav_msgs::msg::Path path;
    path.header = command.header;
    path.poses.reserve(result.trajectory.size());
    for (const auto & sample : result.trajectory) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = sample.x;
      pose.pose.position.y = sample.y;
      pose.pose.orientation.z = std::sin(sample.yaw * 0.5);
      pose.pose.orientation.w = std::cos(sample.yaw * 0.5);
      path.poses.push_back(pose);
    }
    trajectory_pub_->publish(path);
  }

  std::unique_ptr<LocalAvoidance> planner_;
  std::vector<Point2> obstacles_;
  Point2 goal_;
  bool have_obstacles_{false};
  bool have_waypoint_{false};
  std::chrono::steady_clock::time_point obstacle_receive_time_{};
  std::chrono::steady_clock::time_point waypoint_receive_time_{};
  std::chrono::duration<double> input_timeout_{0.30};
  double publish_rate_hz_{20.0};
  std::string frame_id_{"base_link"};

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ugv_subject1_avoidance_mvp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv_subject1_avoidance_mvp::LocalAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}
