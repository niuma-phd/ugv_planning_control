#include "ugv_subject1_perception_mvp/grid_extractor.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace ugv_subject1_perception_mvp
{
namespace
{

bool host_is_big_endian()
{
  const std::uint16_t value = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01;
}

template<typename T>
T read_scalar(const std::uint8_t * data, bool swap)
{
  T value{};
  std::memcpy(&value, data, sizeof(T));
  if (swap) {
    auto * bytes = reinterpret_cast<std::uint8_t *>(&value);
    std::reverse(bytes, bytes + sizeof(T));
  }
  return value;
}

std::optional<std::uint32_t> xyz_offset(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  std::optional<std::uint32_t> offset;
  for (const auto & field : cloud.fields) {
    if (field.name != name) {
      continue;
    }
    if (offset || field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1) {
      return std::nullopt;
    }
    offset = field.offset;
  }
  return offset;
}

}  // namespace

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode()
  : Node("obstacle_detector_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    extractor_(load_parameters())
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    input_timeout_s_ = declare_parameter<double>("input_timeout_s", 0.5);
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.1);
    min_finite_points_ = declare_parameter<int>("min_finite_points", 1);
    if (base_frame_.empty() || !std::isfinite(input_timeout_s_) ||
      !std::isfinite(tf_timeout_s_) || input_timeout_s_ <= 0.0 ||
      tf_timeout_s_ < 0.0 || min_finite_points_ <= 0)
    {
      throw std::invalid_argument(
              "base_frame, finite timeouts, and a positive min_finite_points are required");
    }

    obstacles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/subject1/obstacles", rclcpp::QoS(1).reliable());
    detected_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/subject1/obstacle_detected", rclcpp::QoS(1).reliable());
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleDetectorNode::on_cloud, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ObstacleDetectorNode::on_timer, this));
  }

private:
  GridParameters load_parameters()
  {
    GridParameters p;
    p.roi_min_x = declare_parameter<double>("roi_min_x", p.roi_min_x);
    p.roi_max_x = declare_parameter<double>("roi_max_x", p.roi_max_x);
    p.roi_min_y = declare_parameter<double>("roi_min_y", p.roi_min_y);
    p.roi_max_y = declare_parameter<double>("roi_max_y", p.roi_max_y);
    p.min_z = declare_parameter<double>("min_z", p.min_z);
    p.max_z = declare_parameter<double>("max_z", p.max_z);
    p.self_min_x = declare_parameter<double>("self_min_x", p.self_min_x);
    p.self_max_x = declare_parameter<double>("self_max_x", p.self_max_x);
    p.self_min_y = declare_parameter<double>("self_min_y", p.self_min_y);
    p.self_max_y = declare_parameter<double>("self_max_y", p.self_max_y);
    p.cell_size = declare_parameter<double>("cell_size", p.cell_size);
    p.min_points = declare_parameter<int>("min_points", p.min_points);
    p.corridor_min_x = declare_parameter<double>("corridor_min_x", p.corridor_min_x);
    p.corridor_max_x = declare_parameter<double>("corridor_max_x", p.corridor_max_x);
    p.corridor_half_width =
      declare_parameter<double>("corridor_half_width", p.corridor_half_width);
    return p;
  }

  bool validate_layout(
    const sensor_msgs::msg::PointCloud2 & cloud, std::uint32_t x, std::uint32_t y,
    std::uint32_t z) const
  {
    if (cloud.height == 0 || cloud.width == 0 || cloud.point_step == 0) {
      return false;
    }
    const std::uint64_t packed_row =
      static_cast<std::uint64_t>(cloud.point_step) * cloud.width;
    const std::uint64_t expected_data = static_cast<std::uint64_t>(cloud.row_step) * cloud.height;
    if (packed_row > cloud.row_step || expected_data != cloud.data.size()) {
      return false;
    }
    if (x == y || x == z || y == z) {
      return false;
    }
    for (const auto offset : {x, y, z}) {
      if (static_cast<std::uint64_t>(offset) + sizeof(float) > cloud.point_step) {
        return false;
      }
    }
    return true;
  }

  void reject_cloud(const std::string & reason)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected point cloud: %s", reason.c_str());
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    const auto x = xyz_offset(*cloud, "x");
    const auto y = xyz_offset(*cloud, "y");
    const auto z = xyz_offset(*cloud, "z");
    if (!x || !y || !z || !validate_layout(*cloud, *x, *y, *z)) {
      reject_cloud("invalid x/y/z fields or buffer layout");
      return;
    }
    if (cloud->header.frame_id.empty()) {
      reject_cloud("empty source frame");
      return;
    }
    const auto stamp_ns = rclcpp::Time(cloud->header.stamp).nanoseconds();
    if (stamp_ns <= 0) {
      reject_cloud("zero or negative source timestamp");
      return;
    }
    if (last_valid_cloud_stamp_ns_ && stamp_ns <= *last_valid_cloud_stamp_ns_) {
      reject_cloud("source timestamp did not advance");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        base_frame_, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_s_));
    } catch (const tf2::TransformException & error) {
      reject_cloud(std::string("TF unavailable: ") + error.what());
      return;
    }

    const auto & q_msg = transform.transform.rotation;
    if (!std::isfinite(q_msg.x) || !std::isfinite(q_msg.y) || !std::isfinite(q_msg.z) ||
      !std::isfinite(q_msg.w) || !std::isfinite(transform.transform.translation.x) ||
      !std::isfinite(transform.transform.translation.y) ||
      !std::isfinite(transform.transform.translation.z))
    {
      reject_cloud("non-finite TF");
      return;
    }
    tf2::Quaternion rotation(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    if (rotation.length2() < 1e-12) {
      reject_cloud("invalid TF quaternion");
      return;
    }
    rotation.normalize();
    const tf2::Transform tf(
      rotation, tf2::Vector3(
        transform.transform.translation.x, transform.transform.translation.y,
        transform.transform.translation.z));

    std::vector<Point3> points;
    points.reserve(static_cast<std::size_t>(cloud->width) * cloud->height);
    const bool swap = cloud->is_bigendian != host_is_big_endian();
    for (std::uint32_t row = 0; row < cloud->height; ++row) {
      const auto row_offset = static_cast<std::size_t>(row) * cloud->row_step;
      for (std::uint32_t col = 0; col < cloud->width; ++col) {
        const auto offset = row_offset + static_cast<std::size_t>(col) * cloud->point_step;
        const float px = read_scalar<float>(&cloud->data[offset + *x], swap);
        const float py = read_scalar<float>(&cloud->data[offset + *y], swap);
        const float pz = read_scalar<float>(&cloud->data[offset + *z], swap);
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
          continue;
        }
        const auto transformed = tf * tf2::Vector3(px, py, pz);
        if (!std::isfinite(transformed.x()) || !std::isfinite(transformed.y()) ||
          !std::isfinite(transformed.z()))
        {
          continue;
        }
        points.push_back({transformed.x(), transformed.y(), transformed.z()});
      }
    }
    if (points.size() < static_cast<std::size_t>(min_finite_points_)) {
      reject_cloud("too few finite points");
      return;
    }

    const auto result = extractor_.extract(points);
    geometry_msgs::msg::PoseArray obstacles;
    obstacles.header.stamp = cloud->header.stamp;
    obstacles.header.frame_id = base_frame_;
    obstacles.poses.reserve(result.occupied_centers.size());
    for (const auto & center : result.occupied_centers) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = center.x;
      pose.position.y = center.y;
      pose.position.z = center.z;
      pose.orientation.w = 1.0;
      obstacles.poses.push_back(pose);
    }
    obstacles_pub_->publish(obstacles);
    std_msgs::msg::Bool detected;
    detected.data = result.obstacle_detected;
    detected_pub_->publish(detected);
    last_valid_cloud_ = now();
    last_valid_cloud_stamp_ns_ = stamp_ns;
  }

  void on_timer()
  {
    if (!last_valid_cloud_ || (now() - *last_valid_cloud_).seconds() > input_timeout_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no valid point cloud within %.3f s; withholding output so avoidance fails closed",
        input_timeout_s_);
    }
  }

  std::string base_frame_;
  double input_timeout_s_{0.5};
  double tf_timeout_s_{0.1};
  int min_finite_points_{1};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  GridExtractor extractor_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<rclcpp::Time> last_valid_cloud_;
  std::optional<std::int64_t> last_valid_cloud_stamp_ns_;
};

}  // namespace ugv_subject1_perception_mvp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv_subject1_perception_mvp::ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
