#include <functional>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ugv_localization_mvp/ros_time.hpp"
#include "ugv_localization_mvp/transform_math.hpp"

namespace ugv_localization_mvp
{

class LioOdomAdapterNode : public rclcpp::Node
{
public:
  LioOdomAdapterNode() : Node("lio_odom_adapter")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/livox_odometry_mapped");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/localization/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    raw_world_frame_ = declare_parameter<std::string>("raw_world_frame", "world");
    raw_lidar_frame_ = declare_parameter<std::string>("raw_lidar_frame", "livox_frame");
    timestamp_mode_ = declare_parameter<std::string>("timestamp_mode", "source");
    if (timestamp_mode_ != "source" && timestamp_mode_ != "relative_to_ros") {
      throw std::invalid_argument(
              "timestamp_mode must be 'source' or 'relative_to_ros'");
    }
    extrinsics_valid_ = declare_parameter<bool>("extrinsics_valid", false);
    base_lidar_ = makeTransform(
      declare_parameter<double>("base_to_lidar.x", 0.0),
      declare_parameter<double>("base_to_lidar.y", 0.0),
      declare_parameter<double>("base_to_lidar.z", 0.0),
      declare_parameter<double>("base_to_lidar.roll", 0.0),
      declare_parameter<double>("base_to_lidar.pitch", 0.0),
      declare_parameter<double>("base_to_lidar.yaw", 0.0));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    auto input_qos = rclcpp::SensorDataQoS();
    input_qos.keep_last(1);
    raw_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic, input_qos,
      std::bind(&LioOdomAdapterNode::onOdom, this, std::placeholders::_1));

    if (timestamp_mode_ == "relative_to_ros") {
      RCLCPP_WARN(
        get_logger(),
        "timestamp_mode=relative_to_ros: anchoring the relative source clock to ROS time; "
        "source stamps must remain strictly increasing");
    }

    if (!extrinsics_valid_) {
      RCLCPP_ERROR(
        get_logger(),
        "base_to_lidar is not approved (extrinsics_valid=false); canonical odom and TF are disabled");
    }
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!extrinsics_valid_) {return;}
    const auto source_stamp_ns = positiveRosTimeToNanoseconds(msg->header.stamp);
    if (!source_stamp_ns) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "raw odom timestamp is invalid; dropping");
      return;
    }
    if (msg->header.frame_id != raw_world_frame_ || msg->child_frame_id != raw_lidar_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "raw odom frames are '%s' -> '%s', expected '%s' -> '%s'; dropping",
        msg->header.frame_id.c_str(), msg->child_frame_id.c_str(), raw_world_frame_.c_str(),
        raw_lidar_frame_.c_str());
      return;
    }
    if (!finiteAndNormalized(msg->pose.pose) || !finiteAndNormalized(base_lidar_)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "raw pose or approved extrinsic is invalid; dropping");
      return;
    }

    const auto transform = odomBaseFromRawLidar(msg->pose.pose, base_lidar_);
    if (!finiteAndNormalized(transform)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "computed odom pose is invalid; dropping");
      return;
    }

    nav_msgs::msg::Odometry output;
    output.header.stamp = msg->header.stamp;
    if (timestamp_mode_ == "relative_to_ros") {
      const auto mapped_stamp_ns = relative_time_mapper_.map(
        *source_stamp_ns, now().nanoseconds());
      if (!mapped_stamp_ns) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "relative raw odom timestamp did not advance or could not be mapped; dropping");
        return;
      }
      output.header.stamp = rclcpp::Time(
        *mapped_stamp_ns, get_clock()->get_clock_type());
    }
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = transform.translation.x;
    output.pose.pose.position.y = transform.translation.y;
    output.pose.pose.position.z = transform.translation.z;
    output.pose.pose.orientation = transform.rotation;
    output.pose.covariance = msg->pose.covariance;
    // The current LIO does not provide a trustworthy base-frame twist. Leave it explicitly zero.
    odom_pub_->publish(output);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = output.header;
    tf.child_frame_id = base_frame_;
    tf.transform = transform;
    tf_broadcaster_->sendTransform(tf);
  }

  std::string odom_frame_;
  std::string base_frame_;
  std::string raw_world_frame_;
  std::string raw_lidar_frame_;
  std::string timestamp_mode_;
  bool extrinsics_valid_{false};
  geometry_msgs::msg::Transform base_lidar_;
  RelativeRosTimeMapper relative_time_mapper_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace ugv_localization_mvp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv_localization_mvp::LioOdomAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
