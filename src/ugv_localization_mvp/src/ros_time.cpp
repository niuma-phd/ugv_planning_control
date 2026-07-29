#include "ugv_localization_mvp/ros_time.hpp"

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

}  // namespace ugv_localization_mvp
