#include <functional>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ugv_localization_mvp/odom_guard_core.hpp"

namespace ugv_localization_mvp
{
namespace
{
template<typename RangeT>
bool allFinite(const RangeT & values)
{
  for (const auto value : values) {
    if (!std::isfinite(value)) {return false;}
  }
  return true;
}

std::string jsonEscape(const std::string & input)
{
  std::ostringstream output;
  for (const char c : input) {
    if (c == '\\' || c == '"') {output << '\\';}
    else if (c == '\n') {output << "\\n";}
    else {output << c;}
  }
  return output.str();
}

void replaceAtomically(const std::filesystem::path & destination, const std::string & contents)
{
  const auto temporary = destination.string() + ".tmp";
  {
    std::ofstream stream(temporary, std::ios::out | std::ios::trunc);
    if (!stream) {throw std::runtime_error("cannot open temporary snapshot " + temporary);}
    stream << contents;
    stream.flush();
    if (!stream) {throw std::runtime_error("cannot write temporary snapshot " + temporary);}
  }
  std::filesystem::rename(temporary, destination);
}
}  // namespace

class OdomGuardNode : public rclcpp::Node
{
public:
  OdomGuardNode()
  : Node("odom_guard"), core_(loadSettings())
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/localization/odom");
    trusted_topic_ = declare_parameter<std::string>(
      "trusted_topic", "/localization/trusted_odom");
    valid_topic_ = declare_parameter<std::string>("valid_topic", "/localization/odom_valid");
    snapshot_directory_ = declare_parameter<std::string>("snapshot_directory", "/tmp/ugv_odom_guard");
    snapshot_basename_ = declare_parameter<std::string>("snapshot_basename", "last_good_odom");
    const double watchdog_rate_hz = declare_parameter<double>("watchdog_rate_hz", 20.0);
    if (watchdog_rate_hz <= 0.0) {throw std::invalid_argument("watchdog_rate_hz must be positive");}
    std::filesystem::create_directories(snapshot_directory_);

    trusted_pub_ = create_publisher<nav_msgs::msg::Odometry>(trusted_topic_, rclcpp::QoS(10));
    last_good_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/localization/last_trusted_odom", rclcpp::QoS(1).transient_local());
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      valid_topic_, rclcpp::QoS(1).reliable().transient_local());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&OdomGuardNode::onOdom, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/reset_odom_fault",
      std::bind(&OdomGuardNode::onReset, this, std::placeholders::_1, std::placeholders::_2));
    watchdog_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / watchdog_rate_hz),
      std::bind(&OdomGuardNode::onWatchdog, this));
    publishValid(false);
  }

private:
  OdomGuardSettings loadSettings()
  {
    OdomGuardSettings settings;
    settings.max_age_s = declare_parameter<double>("max_age_s", settings.max_age_s);
    max_age_s_ = settings.max_age_s;
    settings.future_tolerance_s = declare_parameter<double>(
      "future_tolerance_s", settings.future_tolerance_s);
    settings.quaternion_norm_tolerance = declare_parameter<double>(
      "quaternion_norm_tolerance", settings.quaternion_norm_tolerance);
    settings.max_translation_jump_m = declare_parameter<double>(
      "max_translation_jump_m", settings.max_translation_jump_m);
    settings.max_yaw_jump_rad = declare_parameter<double>(
      "max_yaw_jump_rad", settings.max_yaw_jump_rad);
    return settings;
  }

  static OdomSample sampleFrom(const nav_msgs::msg::Odometry & msg)
  {
    OdomSample sample;
    sample.stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
    sample.x = msg.pose.pose.position.x;
    sample.y = msg.pose.pose.position.y;
    sample.z = msg.pose.pose.position.z;
    sample.qx = msg.pose.pose.orientation.x;
    sample.qy = msg.pose.pose.orientation.y;
    sample.qz = msg.pose.pose.orientation.z;
    sample.qw = msg.pose.pose.orientation.w;
    sample.vx = msg.twist.twist.linear.x;
    sample.vy = msg.twist.twist.linear.y;
    sample.vz = msg.twist.twist.linear.z;
    sample.wx = msg.twist.twist.angular.x;
    sample.wy = msg.twist.twist.angular.y;
    sample.wz = msg.twist.twist.angular.z;
    sample.auxiliary_finite = allFinite(msg.pose.covariance) && allFinite(msg.twist.covariance);
    return sample;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const OdomSample sample = sampleFrom(*msg);
    last_received_sample_ = sample;
    have_received_sample_ = true;
    const OdomFault fault = core_.evaluate(sample, now().nanoseconds());
    if (fault != OdomFault::kNone) {
      handleFault(fault);
      return;
    }
    last_good_msg_ = *msg;
    have_last_good_msg_ = true;
    trusted_pub_->publish(*msg);
    last_good_pub_->publish(*msg);
    publishValid(true);
  }

  void onWatchdog()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (core_.latched() || !have_received_sample_) {return;}
    const double age_s = static_cast<double>(now().nanoseconds() - last_received_sample_.stamp_ns) / 1.0e9;
    if (age_s <= max_age_s_) {return;}
    const OdomFault fault = core_.evaluate(last_received_sample_, now().nanoseconds());
    if (fault != OdomFault::kNone) {handleFault(fault);}
  }

  void handleFault(OdomFault fault)
  {
    publishValid(false);
    if (snapshot_written_) {return;}
    snapshot_written_ = true;
    if (!have_last_good_msg_) {
      RCLCPP_ERROR(get_logger(), "odometry fault latched (%s) before any trusted sample", toString(fault));
      return;
    }
    try {
      writeSnapshot(last_good_msg_, fault);
      RCLCPP_ERROR(
        get_logger(), "odometry fault latched (%s); last-good snapshot saved in %s",
        toString(fault), snapshot_directory_.c_str());
    } catch (const std::exception & error) {
      RCLCPP_FATAL(
        get_logger(), "odometry fault latched (%s), but snapshot persistence failed: %s",
        toString(fault), error.what());
    }
  }

  void writeSnapshot(const nav_msgs::msg::Odometry & msg, OdomFault fault)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const auto & linear = msg.twist.twist.linear;
    const auto & angular = msg.twist.twist.angular;
    const std::int64_t stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
    std::ostringstream json;
    json << std::setprecision(17)
         << "{\n  \"fault\": \"" << toString(fault) << "\",\n"
         << "  \"stamp_ns\": " << stamp_ns << ",\n"
         << "  \"frame_id\": \"" << jsonEscape(msg.header.frame_id) << "\",\n"
         << "  \"child_frame_id\": \"" << jsonEscape(msg.child_frame_id) << "\",\n"
         << "  \"position_m\": [" << p.x << ", " << p.y << ", " << p.z << "],\n"
         << "  \"orientation_xyzw\": [" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "],\n"
         << "  \"linear_velocity_mps\": [" << linear.x << ", " << linear.y << ", " << linear.z << "],\n"
         << "  \"angular_velocity_radps\": [" << angular.x << ", " << angular.y << ", " << angular.z << "]\n}\n";
    std::ostringstream csv;
    csv << "fault,stamp_ns,frame_id,child_frame_id,x_m,y_m,z_m,qx,qy,qz,qw,vx_mps,vy_mps,vz_mps,wx_radps,wy_radps,wz_radps\n"
        << std::setprecision(17) << toString(fault) << ',' << stamp_ns << ','
        << msg.header.frame_id << ',' << msg.child_frame_id << ','
        << p.x << ',' << p.y << ',' << p.z << ',' << q.x << ',' << q.y << ',' << q.z << ',' << q.w << ','
        << linear.x << ',' << linear.y << ',' << linear.z << ','
        << angular.x << ',' << angular.y << ',' << angular.z << '\n';
    const std::filesystem::path directory(snapshot_directory_);
    replaceAtomically(directory / (snapshot_basename_ + ".json"), json.str());
    replaceAtomically(directory / (snapshot_basename_ + ".csv"), csv.str());
  }

  void onReset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    core_.reset();
    have_received_sample_ = false;
    have_last_good_msg_ = false;
    snapshot_written_ = false;
    publishValid(false);
    response->success = true;
    response->message = "fault reset; waiting for a new valid odometry sample";
  }

  void publishValid(bool valid)
  {
    std_msgs::msg::Bool message;
    message.data = valid;
    valid_pub_->publish(message);
  }

  std::string input_topic_;
  std::string trusted_topic_;
  std::string valid_topic_;
  std::string snapshot_directory_;
  std::string snapshot_basename_;
  std::mutex mutex_;
  double max_age_s_{0.30};
  OdomGuardCore core_;
  bool have_received_sample_{false};
  bool have_last_good_msg_{false};
  bool snapshot_written_{false};
  OdomSample last_received_sample_{};
  nav_msgs::msg::Odometry last_good_msg_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr trusted_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr last_good_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace ugv_localization_mvp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv_localization_mvp::OdomGuardNode>());
  rclcpp::shutdown();
  return 0;
}
