#include "ugv_localization_mvp/ros_time.hpp"

#include <limits>

namespace ugv_localization_mvp
{

std::optional<std::int64_t> positiveRosTimeToNanoseconds(
  const builtin_interfaces::msg::Time & stamp) noexcept
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
    return std::nullopt;
  }
  const auto nanoseconds =
    static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(stamp.nanosec);
  if (nanoseconds <= 0) {
    return std::nullopt;
  }
  return nanoseconds;
}

std::optional<std::int64_t> RelativeRosTimeMapper::map(
  std::int64_t source_stamp_ns, std::int64_t reception_stamp_ns) noexcept
{
  if (source_stamp_ns <= 0 || reception_stamp_ns <= 0) {
    return std::nullopt;
  }
  if (!initialized_) {
    initialized_ = true;
    source_origin_ns_ = source_stamp_ns;
    ros_origin_ns_ = reception_stamp_ns;
    last_source_stamp_ns_ = source_stamp_ns;
    return ros_origin_ns_;
  }
  if (source_stamp_ns <= last_source_stamp_ns_) {
    return std::nullopt;
  }
  const std::int64_t source_delta_ns = source_stamp_ns - source_origin_ns_;
  if (source_delta_ns > std::numeric_limits<std::int64_t>::max() - ros_origin_ns_) {
    return std::nullopt;
  }
  last_source_stamp_ns_ = source_stamp_ns;
  return ros_origin_ns_ + source_delta_ns;
}

void RelativeRosTimeMapper::reset() noexcept
{
  initialized_ = false;
  source_origin_ns_ = 0;
  ros_origin_ns_ = 0;
  last_source_stamp_ns_ = 0;
}

}  // namespace ugv_localization_mvp
