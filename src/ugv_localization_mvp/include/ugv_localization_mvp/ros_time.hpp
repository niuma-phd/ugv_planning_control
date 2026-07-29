#pragma once

#include <cstdint>
#include <optional>

#include "builtin_interfaces/msg/time.hpp"

namespace ugv_localization_mvp
{

std::optional<std::int64_t> positiveRosTimeToNanoseconds(
  const builtin_interfaces::msg::Time & stamp) noexcept;

// Maps a strictly increasing relative-device clock into the ROS clock domain.
// The first source stamp is anchored to its ROS reception time; later stamps
// preserve source-clock deltas so downstream freshness checks still detect
// processing stalls instead of receiving a fresh timestamp on every callback.
class RelativeRosTimeMapper
{
public:
  std::optional<std::int64_t> map(
    std::int64_t source_stamp_ns, std::int64_t reception_stamp_ns) noexcept;
  void reset() noexcept;

private:
  bool initialized_{false};
  std::int64_t source_origin_ns_{0};
  std::int64_t ros_origin_ns_{0};
  std::int64_t last_source_stamp_ns_{0};
};

}  // namespace ugv_localization_mvp
