#pragma once

#include <cstdint>
#include <optional>

#include "builtin_interfaces/msg/time.hpp"

namespace ugv_localization_mvp
{

std::optional<std::int64_t> positiveRosTimeToNanoseconds(
  const builtin_interfaces::msg::Time & stamp) noexcept;

}  // namespace ugv_localization_mvp
