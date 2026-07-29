#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ugv_localization_mvp/recovery_core.hpp"
#include "ugv_localization_mvp/ros_time.hpp"

namespace ugv_localization_mvp
{

class RecoveryCoordinatorNode : public rclcpp::Node
{
public:
  RecoveryCoordinatorNode()
  : Node("recovery_coordinator"), settings_(loadSettings()),
    gps_window_(
      settings_.gps_sample_count, settings_.max_gps_window_position_span_m,
      settings_.max_gps_window_yaw_span_rad),
    odom_window_(
      settings_.odom_sample_count, settings_.max_odom_window_position_span_m,
      settings_.max_odom_window_yaw_span_rad)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    automatic_recovery_enabled_ = declare_parameter<bool>("automatic_recovery_enabled", false);
    heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 10.0);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 1.0);
    future_stamp_tolerance_sec_ = declare_parameter<double>("future_stamp_tolerance_sec", 0.1);
    recovery_timeout_sec_ = declare_parameter<double>("recovery_timeout_sec", 20.0);
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 15.0);
    startup_invalid_grace_sec_ = declare_parameter<double>(
      "startup_invalid_grace_sec", 0.25);
    max_alignment_stamp_skew_sec_ = declare_parameter<double>(
      "max_alignment_stamp_skew_sec", 0.20);
    echo_translation_tolerance_m_ = declare_parameter<double>(
      "echo_translation_tolerance_m", 0.02);
    echo_yaw_tolerance_rad_ = declare_parameter<double>("echo_yaw_tolerance_rad", 0.02);
    final_position_tolerance_m_ = declare_parameter<double>("final_position_tolerance_m", 1.0);
    final_yaw_tolerance_rad_ = declare_parameter<double>("final_yaw_tolerance_rad", 0.35);
    startup_odom_samples_ = positiveSizeParameter("startup_odom_samples", 5);
    max_recovery_attempts_ = positiveSizeParameter("max_recovery_attempts", 1);

    if (map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
      map_frame_ == odom_frame_ || map_frame_ == base_frame_ || odom_frame_ == base_frame_ ||
      !finitePositive(heartbeat_rate_hz_) || !finitePositive(input_timeout_sec_) ||
      !std::isfinite(future_stamp_tolerance_sec_) || future_stamp_tolerance_sec_ < 0.0 ||
      !finitePositive(recovery_timeout_sec_) || !finitePositive(service_timeout_sec_) ||
      !finitePositive(startup_invalid_grace_sec_) ||
      startup_invalid_grace_sec_ >= input_timeout_sec_ ||
      !finitePositive(max_alignment_stamp_skew_sec_) ||
      !finitePositive(echo_translation_tolerance_m_) ||
      !finitePositive(echo_yaw_tolerance_rad_) || !finitePositive(final_position_tolerance_m_) ||
      !finitePositive(final_yaw_tolerance_rad_))
    {
      throw std::invalid_argument(
              "recovery coordinator frames, rates, timeouts, and tolerances must be valid");
    }

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    navigation_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/localization/navigation_enabled", latched_qos);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "/localization/recovery_state", latched_qos);
    map_update_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
      "/localization/map_odom_update", rclcpp::QoS(1).reliable());

    gps_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/gps_pose", rclcpp::QoS(10).reliable(),
      std::bind(&RecoveryCoordinatorNode::onGpsPose, this, std::placeholders::_1));
    gps_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization/gps_valid", rclcpp::QoS(10).reliable(),
      std::bind(&RecoveryCoordinatorNode::onGpsValid, this, std::placeholders::_1));
    odom_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization/odom_valid", latched_qos,
      std::bind(&RecoveryCoordinatorNode::onOdomValid, this, std::placeholders::_1));
    last_trusted_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/last_trusted_odom", latched_qos,
      std::bind(&RecoveryCoordinatorNode::onLastTrusted, this, std::placeholders::_1));
    raw_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/odom", rclcpp::QoS(10).reliable(),
      std::bind(&RecoveryCoordinatorNode::onRawOdom, this, std::placeholders::_1));
    map_odom_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      "/localization/map_odom", latched_qos,
      std::bind(&RecoveryCoordinatorNode::onMapOdom, this, std::placeholders::_1));
    lio_generation_sub_ = create_subscription<std_msgs::msg::UInt32>(
      "/localization/lio_generation", latched_qos,
      std::bind(&RecoveryCoordinatorNode::onLioGeneration, this, std::placeholders::_1));
    lio_alive_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization/lio_process_alive", latched_qos,
      std::bind(&RecoveryCoordinatorNode::onLioAlive, this, std::placeholders::_1));

    restart_client_ = create_client<std_srvs::srv::Trigger>("/localization/restart_lio");
    reset_client_ = create_client<std_srvs::srv::Trigger>("/localization/reset_odom_fault");
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / heartbeat_rate_hz_),
      std::bind(&RecoveryCoordinatorNode::onTimer, this));
    phase_started_at_ = steadyNow();
    publishStatus();
  }

private:
  enum class State {kInitializing, kRunning, kWaitGps, kWaitLio, kAborted};
  enum class LioPhase {kNone, kCollecting, kWaitEcho, kWaitReset, kWaitGuard};

  static bool finitePositive(double value) {return std::isfinite(value) && value > 0.0;}

  std::size_t positiveSizeParameter(const std::string & name, int default_value)
  {
    const auto value = declare_parameter<int>(name, default_value);
    if (value <= 0) {throw std::invalid_argument(name + " must be a positive integer");}
    return static_cast<std::size_t>(value);
  }

  RecoveryCoreSettings loadSettings()
  {
    RecoveryCoreSettings settings;
    settings.gps_sample_count = positiveSizeParameter("gps_sample_count", 5);
    settings.odom_sample_count = positiveSizeParameter("odom_sample_count", 5);
    settings.max_gps_position_covariance = declare_parameter<double>(
      "max_gps_position_covariance", settings.max_gps_position_covariance);
    settings.max_gps_yaw_covariance = declare_parameter<double>(
      "max_gps_yaw_covariance", settings.max_gps_yaw_covariance);
    settings.max_gps_window_position_span_m = declare_parameter<double>(
      "max_gps_window_position_span_m", settings.max_gps_window_position_span_m);
    settings.max_gps_window_yaw_span_rad = declare_parameter<double>(
      "max_gps_window_yaw_span_rad", settings.max_gps_window_yaw_span_rad);
    settings.max_odom_window_position_span_m = declare_parameter<double>(
      "max_odom_window_position_span_m", settings.max_odom_window_position_span_m);
    settings.max_odom_window_yaw_span_rad = declare_parameter<double>(
      "max_odom_window_yaw_span_rad", settings.max_odom_window_yaw_span_rad);
    settings.max_anchor_correction_m = declare_parameter<double>(
      "max_anchor_correction_m", settings.max_anchor_correction_m);
    settings.max_anchor_correction_yaw_rad = declare_parameter<double>(
      "max_anchor_correction_yaw_rad", settings.max_anchor_correction_yaw_rad);
    if (!validRecoverySettings(settings)) {
      throw std::invalid_argument("all recovery core settings must be finite and positive");
    }
    return settings;
  }

  std::chrono::steady_clock::time_point steadyNow() const
  {
    return std::chrono::steady_clock::now();
  }

  bool recent(const std::chrono::steady_clock::time_point & received_at) const
  {
    return std::chrono::duration<double>(steadyNow() - received_at).count() <= input_timeout_sec_;
  }

  bool freshRosStamp(std::int64_t stamp_ns) const
  {
    const auto now_ns = now().nanoseconds();
    if (stamp_ns <= 0 || now_ns <= 0) {return false;}
    const double age_sec = static_cast<double>(now_ns - stamp_ns) / 1.0e9;
    return age_sec <= input_timeout_sec_ && age_sec >= -future_stamp_tolerance_sec_;
  }

  const char * stateName() const
  {
    switch (state_) {
      case State::kInitializing: return "INITIALIZING";
      case State::kRunning: return "RUNNING";
      case State::kWaitGps: return "WAIT_GPS";
      case State::kWaitLio: return "WAIT_LIO";
      case State::kAborted: return "ABORTED";
    }
    return "ABORTED";
  }

  void publishStatus()
  {
    std_msgs::msg::Bool enabled;
    enabled.data = state_ == State::kRunning;
    navigation_pub_->publish(enabled);
    std_msgs::msg::String state;
    state.data = stateName();
    state_pub_->publish(state);
  }

  void abort(const std::string & reason)
  {
    if (state_ == State::kAborted) {return;}
    state_ = State::kAborted;
    lio_phase_ = LioPhase::kNone;
    RCLCPP_ERROR(get_logger(), "localization recovery aborted: %s", reason.c_str());
    publishStatus();
  }

  void onGpsValid(const std_msgs::msg::Bool::SharedPtr msg)
  {
    gps_valid_ = msg->data;
    gps_valid_received_ = true;
    gps_valid_received_at_ = steadyNow();
    if (!gps_valid_) {
      gps_window_.clear();
      recovery_gps_anchor_.reset();
    }
  }

  void onGpsPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const bool collecting_before_restart = state_ == State::kWaitGps && !restart_request_sent_;
    const bool collecting_after_restart =
      state_ == State::kWaitLio && lio_phase_ == LioPhase::kCollecting;
    if (!collecting_before_restart && !collecting_after_restart) {return;}
    if (!gps_valid_received_ || !gps_valid_ || !recent(gps_valid_received_at_)) {return;}
    if (msg->header.frame_id != map_frame_) {
      abort("GPS pose frame must be map");
      return;
    }
    const auto stamp_ns = positiveRosTimeToNanoseconds(msg->header.stamp);
    if (!stamp_ns) {
      abort("GPS pose timestamp is invalid");
      return;
    }
    if (!freshRosStamp(*stamp_ns)) {
      abort("GPS pose timestamp is stale or too far in the future");
      return;
    }
    RecoveryPose pose;
    std::string reason;
    if (!recoveryPoseFromGps(msg->pose, *stamp_ns, settings_, pose, reason)) {
      abort(reason);
      return;
    }
    gps_pose_received_at_ = steadyNow();
    gps_window_.add(pose);
    if (!gps_window_.ready()) {
      recovery_gps_anchor_.reset();
      return;
    }
    if (!last_fault_odom_ || !map_odom_) {
      abort("last trusted odometry or current map->odom anchor is unavailable");
      return;
    }
    if (!anchorCorrectionAcceptable(
        gps_window_.latest(), *last_fault_odom_, map_odom_->transform, settings_, reason))
    {
      abort(reason);
      return;
    }
    recovery_gps_anchor_ = gps_window_.latest();
    if (collecting_before_restart) {
      requestRestart();
    } else {
      maybePublishAlignment();
    }
  }

  bool odomPoseFromMessage(
    const nav_msgs::msg::Odometry & msg, RecoveryPose & pose, std::string & reason) const
  {
    if (msg.header.frame_id != odom_frame_ || msg.child_frame_id != base_frame_) {
      reason = "odometry frames do not match odom->base_link";
      return false;
    }
    const auto stamp_ns = positiveRosTimeToNanoseconds(msg.header.stamp);
    if (!stamp_ns) {
      reason = "odometry timestamp is invalid";
      return false;
    }
    if (!freshRosStamp(*stamp_ns)) {
      reason = "odometry timestamp is stale or too far in the future";
      return false;
    }
    return recoveryPoseFromOdom(msg.pose.pose, *stamp_ns, pose, reason);
  }

  void onLastTrusted(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RecoveryPose pose;
    std::string reason;
    if (!odomPoseFromMessage(*msg, pose, reason)) {
      if (state_ != State::kAborted) {abort("last trusted " + reason);}
      return;
    }
    last_trusted_odom_ = pose;
    last_trusted_received_at_ = steadyNow();
    if (state_ == State::kWaitLio && lio_phase_ == LioPhase::kWaitGuard) {
      tryFinishRecovery();
    }
  }

  void onLioGeneration(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    lio_generation_ = msg->data;
    lio_generation_received_ = true;
    lio_generation_received_at_ = steadyNow();
  }

  void onLioAlive(const std_msgs::msg::Bool::SharedPtr msg)
  {
    lio_process_alive_ = msg->data;
    lio_alive_received_ = true;
    lio_alive_received_at_ = steadyNow();
  }

  bool restartedGenerationReady() const
  {
    return restart_baseline_generation_ && lio_generation_received_ &&
           lio_generation_ != *restart_baseline_generation_ &&
           recent(lio_generation_received_at_) && lio_alive_received_ &&
           lio_process_alive_ && recent(lio_alive_received_at_);
  }

  void onRawOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (state_ != State::kWaitLio || lio_phase_ != LioPhase::kCollecting) {return;}
    if (!restartedGenerationReady()) {return;}
    const auto incoming_stamp_ns = positiveRosTimeToNanoseconds(msg->header.stamp);
    if (incoming_stamp_ns && restart_completed_stamp_ns_ &&
      *incoming_stamp_ns <= *restart_completed_stamp_ns_)
    {
      return;
    }
    RecoveryPose pose;
    std::string reason;
    if (!odomPoseFromMessage(*msg, pose, reason)) {
      abort("new LIO " + reason);
      return;
    }
    // Cross-topic delivery order is not a restart boundary.  Old LIO can keep
    // publishing while the restart service stops its process group, so only
    // odometry newer than the successful service completion reaches the window.
    if (!restart_completed_stamp_ns_) {return;}
    if (last_new_odom_stamp_ns_ && pose.stamp_ns <= *last_new_odom_stamp_ns_) {
      abort("new LIO odometry timestamps are not strictly increasing");
      return;
    }
    last_new_odom_stamp_ns_ = pose.stamp_ns;
    if (startup_samples_remaining_ > 0U) {
      --startup_samples_remaining_;
      return;
    }
    odom_window_.add(pose);
    if (!odom_window_.ready()) {return;}
    maybePublishAlignment();
  }

  void maybePublishAlignment()
  {
    if (state_ != State::kWaitLio || lio_phase_ != LioPhase::kCollecting ||
      !odom_window_.ready() || !gps_window_.ready() || !recovery_gps_anchor_ ||
      !gps_valid_received_ || !gps_valid_ || !recent(gps_valid_received_at_) ||
      !recent(gps_pose_received_at_) || !restartedGenerationReady())
    {
      return;
    }
    const double alignment_stamp_skew_sec = std::abs(
      static_cast<double>(gps_window_.latest().stamp_ns - odom_window_.latest().stamp_ns)) /
      1.0e9;
    if (alignment_stamp_skew_sec > max_alignment_stamp_skew_sec_) {return;}
    // Use the latest post-restart stable GPS sample rather than the sample that
    // initiated the potentially slow restart service.
    recovery_gps_anchor_ = gps_window_.latest();
    aligned_odom_stamp_ns_ = odom_window_.latest().stamp_ns;
    pending_map_odom_ = mapOdomFromRecovery(*recovery_gps_anchor_, odom_window_.latest());
    geometry_msgs::msg::TransformStamped update;
    update.header.stamp = now();
    update.header.frame_id = map_frame_;
    update.child_frame_id = odom_frame_;
    update.transform = *pending_map_odom_;
    map_update_pub_->publish(update);
    lio_phase_ = LioPhase::kWaitEcho;
    phase_started_at_ = steadyNow();
    RCLCPP_INFO(get_logger(), "published recovered map->odom alignment; waiting for manager echo");
  }

  void onMapOdom(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != map_frame_ || msg->child_frame_id != odom_frame_ ||
      !positiveRosTimeToNanoseconds(msg->header.stamp))
    {
      if (state_ != State::kAborted) {abort("map->odom echo has invalid frames or timestamp");}
      return;
    }
    // Retain the message only if it is self-consistent and finite.
    if (!transformsNear(msg->transform, msg->transform, 1.0, 1.0)) {
      abort("map->odom echo transform is invalid");
      return;
    }
    map_odom_ = *msg;
    map_odom_received_at_ = steadyNow();
    if (state_ == State::kWaitLio && lio_phase_ == LioPhase::kWaitEcho &&
      pending_map_odom_ && transformsNear(
        msg->transform, *pending_map_odom_, echo_translation_tolerance_m_,
        echo_yaw_tolerance_rad_))
    {
      requestGuardReset();
    }
  }

  void onOdomValid(const std_msgs::msg::Bool::SharedPtr msg)
  {
    odom_valid_ = msg->data;
    odom_valid_received_ = true;
    odom_valid_received_at_ = steadyNow();
    if (odom_valid_) {
      odom_was_ever_valid_ = true;
      initial_invalid_received_at_.reset();
    } else if (state_ == State::kInitializing && !initial_invalid_received_at_) {
      initial_invalid_received_at_ = odom_valid_received_at_;
    }
    if (state_ == State::kRunning && !odom_valid_ && last_trusted_odom_ && map_odom_)
    {
      beginRecovery();
    } else if (state_ == State::kInitializing && !odom_valid_ && odom_was_ever_valid_ &&
      last_trusted_odom_ && map_odom_)
    {
      beginRecovery();
    } else if (state_ == State::kWaitLio && lio_phase_ == LioPhase::kWaitGuard && odom_valid_) {
      tryFinishRecovery();
    }
  }

  void beginRecovery()
  {
    if (!automatic_recovery_enabled_) {
      abort("automatic recovery is disabled because no complete global pose source is approved");
      return;
    }
    if (recovery_attempts_ >= max_recovery_attempts_) {
      abort("maximum recovery attempts exhausted");
      return;
    }
    if (!last_trusted_odom_ || !recent(last_trusted_received_at_) || !map_odom_ ||
      !recent(map_odom_received_at_))
    {
      abort("fault occurred without a fresh last trusted odometry and map anchor");
      return;
    }
    ++recovery_attempts_;
    last_fault_odom_ = last_trusted_odom_;
    state_ = State::kWaitGps;
    lio_phase_ = LioPhase::kNone;
    gps_window_.clear();
    restart_request_sent_ = false;
    recovery_gps_anchor_.reset();
    pending_map_odom_.reset();
    aligned_odom_stamp_ns_.reset();
    last_new_odom_stamp_ns_.reset();
    restart_baseline_generation_.reset();
    restart_completed_stamp_ns_.reset();
    phase_started_at_ = steadyNow();
    publishStatus();
    RCLCPP_ERROR(
      get_logger(), "odometry became invalid; navigation disabled while waiting for GPS");
  }

  void requestRestart()
  {
    if (restart_request_sent_) {return;}
    if (!recovery_gps_anchor_ || !gps_window_.ready() || !gps_valid_received_ ||
      !gps_valid_ || !recent(gps_valid_received_at_) || !recent(gps_pose_received_at_))
    {
      recovery_gps_anchor_.reset();
      return;
    }
    // A restart service without a latched generation contract cannot prove
    // that subsequent odometry came from a new LIO process.
    if (!lio_generation_received_ || !recent(lio_generation_received_at_) ||
      !restart_client_->service_is_ready())
    {
      return;
    }
    restart_request_sent_ = true;
    restart_baseline_generation_ = lio_generation_;
    service_request_started_at_ = steadyNow();
    phase_started_at_ = service_request_started_at_;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    restart_client_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (state_ != State::kWaitGps) {return;}
        std::shared_ptr<std_srvs::srv::Trigger::Response> response;
        try {
          response = future.get();
        } catch (const std::exception & error) {
          abort(std::string("LIO restart service exception: ") + error.what());
          return;
        }
        if (!response->success) {
          abort("LIO restart service failed: " + response->message);
          return;
        }
        const auto restart_completed_stamp_ns = now().nanoseconds();
        if (restart_completed_stamp_ns <= 0) {
          abort("ROS time is unavailable for the completed LIO restart boundary");
          return;
        }
        restart_completed_stamp_ns_ = restart_completed_stamp_ns;
        state_ = State::kWaitLio;
        lio_phase_ = LioPhase::kCollecting;
        odom_window_.clear();
        gps_window_.clear();
        recovery_gps_anchor_.reset();
        gps_valid_ = false;
        gps_valid_received_ = false;
        startup_samples_remaining_ = startup_odom_samples_;
        last_new_odom_stamp_ns_.reset();
        phase_started_at_ = steadyNow();
        publishStatus();
        RCLCPP_INFO(
          get_logger(),
          "LIO restart accepted; waiting for a new live generation and fresh unguarded odometry");
      });
  }

  void requestGuardReset()
  {
    if (lio_phase_ != LioPhase::kWaitEcho || !reset_client_->service_is_ready()) {return;}
    lio_phase_ = LioPhase::kWaitReset;
    service_request_started_at_ = steadyNow();
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    reset_client_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (state_ != State::kWaitLio || lio_phase_ != LioPhase::kWaitReset) {return;}
        std::shared_ptr<std_srvs::srv::Trigger::Response> response;
        try {
          response = future.get();
        } catch (const std::exception & error) {
          abort(std::string("odometry guard reset service exception: ") + error.what());
          return;
        }
        if (!response->success) {
          abort("odometry guard reset failed: " + response->message);
          return;
        }
        odom_valid_ = false;
        lio_phase_ = LioPhase::kWaitGuard;
        phase_started_at_ = steadyNow();
        RCLCPP_INFO(get_logger(), "odometry guard reset; waiting for a consistent trusted sample");
      });
  }

  void tryFinishRecovery()
  {
    if (!odom_valid_ || !last_trusted_odom_ || !map_odom_ || !recovery_gps_anchor_ ||
      !aligned_odom_stamp_ns_ || last_trusted_odom_->stamp_ns <= *aligned_odom_stamp_ns_)
    {
      return;
    }
    const auto trusted_map = transformPose(map_odom_->transform, *last_trusted_odom_);
    const double position_error = std::hypot(
      std::hypot(
        trusted_map.x - recovery_gps_anchor_->x, trusted_map.y - recovery_gps_anchor_->y),
      trusted_map.z - recovery_gps_anchor_->z);
    const double yaw_error = wrappedAngleDistance(trusted_map.yaw, recovery_gps_anchor_->yaw);
    if (position_error > final_position_tolerance_m_ || yaw_error > final_yaw_tolerance_rad_) {
      abort("new trusted odometry is inconsistent with the GPS recovery anchor");
      return;
    }
    state_ = State::kRunning;
    lio_phase_ = LioPhase::kNone;
    phase_started_at_ = steadyNow();
    publishStatus();
    RCLCPP_INFO(get_logger(), "localization recovery completed; navigation re-enabled");
  }

  void onTimer()
  {
    publishStatus();
    if (state_ == State::kAborted) {return;}
    if (state_ == State::kInitializing) {
      if (odom_valid_received_ && odom_valid_ && recent(odom_valid_received_at_) &&
        last_trusted_odom_ && recent(last_trusted_received_at_) && map_odom_ &&
        recent(map_odom_received_at_))
      {
        state_ = State::kRunning;
        phase_started_at_ = steadyNow();
        publishStatus();
        RCLCPP_INFO(get_logger(), "healthy initial localization received; navigation enabled");
      } else if (odom_valid_received_ && !odom_valid_ && last_trusted_odom_ && map_odom_ &&
        initial_invalid_received_at_)
      {
        const double invalid_age_sec = std::chrono::duration<double>(
          steadyNow() - *initial_invalid_received_at_).count();
        if (initialOdomFaultConfirmed(
            odom_valid_, odom_was_ever_valid_, invalid_age_sec,
            startup_invalid_grace_sec_))
        {
          beginRecovery();
        }
      }
      return;
    }
    if (state_ == State::kRunning) {
      if (!map_odom_ || !recent(map_odom_received_at_)) {
        abort("map->odom heartbeat expired while navigation was enabled");
        return;
      }
      if (!odom_valid_received_ || !odom_valid_ || !recent(odom_valid_received_at_)) {
        beginRecovery();
      }
      return;
    }

    const double phase_age = std::chrono::duration<double>(steadyNow() - phase_started_at_).count();
    if (phase_age > recovery_timeout_sec_) {
      abort("recovery phase timeout");
      return;
    }
    if (state_ == State::kWaitGps && recovery_gps_anchor_ && !restart_request_sent_) {
      requestRestart();
    }
    if (state_ == State::kWaitGps && restart_request_sent_ &&
      std::chrono::duration<double>(steadyNow() - service_request_started_at_).count() >
      service_timeout_sec_)
    {
      abort("LIO restart service response timeout");
      return;
    }
    if (state_ == State::kWaitLio && lio_phase_ == LioPhase::kWaitEcho && pending_map_odom_ &&
      map_odom_ && transformsNear(
        map_odom_->transform, *pending_map_odom_, echo_translation_tolerance_m_,
        echo_yaw_tolerance_rad_))
    {
      requestGuardReset();
    }
    if (state_ == State::kWaitLio && lio_phase_ == LioPhase::kCollecting &&
      gps_valid_received_ &&
      (!gps_valid_ || !recent(gps_valid_received_at_) ||
      (gps_window_.size() > 0U && !recent(gps_pose_received_at_))))
    {
      abort("GPS validity or pose heartbeat expired during post-restart alignment");
      return;
    }
    if (state_ == State::kWaitLio && restart_baseline_generation_ &&
      lio_generation_received_ && lio_generation_ != *restart_baseline_generation_ &&
      lio_alive_received_ && (!lio_process_alive_ || !recent(lio_alive_received_at_)))
    {
      abort("new LIO generation is not alive or its heartbeat expired");
      return;
    }
    if (state_ == State::kWaitLio && lio_phase_ == LioPhase::kWaitReset &&
      std::chrono::duration<double>(steadyNow() - service_request_started_at_).count() >
      service_timeout_sec_)
    {
      abort("odometry guard reset service response timeout");
    }
  }

  RecoveryCoreSettings settings_;
  StablePoseWindow gps_window_;
  StablePoseWindow odom_window_;
  State state_{State::kInitializing};
  LioPhase lio_phase_{LioPhase::kNone};
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool automatic_recovery_enabled_{false};
  double heartbeat_rate_hz_{10.0};
  double input_timeout_sec_{1.0};
  double future_stamp_tolerance_sec_{0.1};
  double recovery_timeout_sec_{20.0};
  double service_timeout_sec_{15.0};
  double startup_invalid_grace_sec_{0.25};
  double max_alignment_stamp_skew_sec_{0.20};
  double echo_translation_tolerance_m_{0.02};
  double echo_yaw_tolerance_rad_{0.02};
  double final_position_tolerance_m_{1.0};
  double final_yaw_tolerance_rad_{0.35};
  std::size_t startup_odom_samples_{5U};
  std::size_t startup_samples_remaining_{0U};
  std::size_t max_recovery_attempts_{1U};
  std::size_t recovery_attempts_{0U};
  bool gps_valid_{false};
  bool gps_valid_received_{false};
  bool odom_valid_{false};
  bool odom_valid_received_{false};
  bool odom_was_ever_valid_{false};
  bool lio_generation_received_{false};
  bool lio_alive_received_{false};
  bool lio_process_alive_{false};
  bool restart_request_sent_{false};
  std::uint32_t lio_generation_{0U};
  std::optional<RecoveryPose> last_trusted_odom_;
  std::optional<RecoveryPose> last_fault_odom_;
  std::optional<RecoveryPose> recovery_gps_anchor_;
  std::optional<geometry_msgs::msg::TransformStamped> map_odom_;
  std::optional<geometry_msgs::msg::Transform> pending_map_odom_;
  std::optional<std::int64_t> last_new_odom_stamp_ns_;
  std::optional<std::int64_t> aligned_odom_stamp_ns_;
  std::optional<std::int64_t> restart_completed_stamp_ns_;
  std::optional<std::uint32_t> restart_baseline_generation_;
  std::chrono::steady_clock::time_point gps_valid_received_at_{};
  std::chrono::steady_clock::time_point gps_pose_received_at_{};
  std::chrono::steady_clock::time_point odom_valid_received_at_{};
  std::chrono::steady_clock::time_point last_trusted_received_at_{};
  std::chrono::steady_clock::time_point map_odom_received_at_{};
  std::chrono::steady_clock::time_point lio_generation_received_at_{};
  std::chrono::steady_clock::time_point lio_alive_received_at_{};
  std::chrono::steady_clock::time_point phase_started_at_{};
  std::chrono::steady_clock::time_point service_request_started_at_{};
  std::optional<std::chrono::steady_clock::time_point> initial_invalid_received_at_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr navigation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr map_update_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gps_valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odom_valid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr last_trusted_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr map_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr lio_generation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lio_alive_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr restart_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ugv_localization_mvp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv_localization_mvp::RecoveryCoordinatorNode>());
  rclcpp::shutdown();
  return 0;
}
