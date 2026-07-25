#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "builtin_interfaces/msg/time.hpp"
#include "ugv_localization_mvp/ros_time.hpp"

namespace ugv_localization_mvp
{

TEST(RosTime, ConvertsPositiveNormalizedStamp)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 34;
  const auto result = positiveRosTimeToNanoseconds(stamp);
  ASSERT_TRUE(result);
  EXPECT_EQ(*result, 12000000034LL);
}

TEST(RosTime, RejectsZeroStamp)
{
  EXPECT_FALSE(positiveRosTimeToNanoseconds(builtin_interfaces::msg::Time{}));
}

TEST(RosTime, RejectsNegativeSeconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = -1;
  EXPECT_FALSE(positiveRosTimeToNanoseconds(stamp));
}

TEST(RosTime, RejectsOutOfRangeNanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1;
  stamp.nanosec = 1000000000U;
  EXPECT_FALSE(positiveRosTimeToNanoseconds(stamp));
}

TEST(RosTime, ConvertsLargestRepresentableSecondsWithoutOverflow)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = std::numeric_limits<std::int32_t>::max();
  stamp.nanosec = 999999999U;
  const auto result = positiveRosTimeToNanoseconds(stamp);
  ASSERT_TRUE(result);
  EXPECT_EQ(*result, 2147483647999999999LL);
}

}  // namespace ugv_localization_mvp
