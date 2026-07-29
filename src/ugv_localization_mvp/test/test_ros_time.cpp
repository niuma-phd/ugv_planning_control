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

TEST(RelativeRosTimeMapper, AnchorsFirstStampAndPreservesSourceDeltas)
{
  RelativeRosTimeMapper mapper;
  const auto first = mapper.map(5 * 1000000000LL, 100 * 1000000000LL);
  const auto second = mapper.map(5200000000LL, 100250000000LL);
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_EQ(*first, 100 * 1000000000LL);
  EXPECT_EQ(*second, 100200000000LL);
}

TEST(RelativeRosTimeMapper, RejectsInvalidOrNonIncreasingSourceStamps)
{
  RelativeRosTimeMapper mapper;
  EXPECT_FALSE(mapper.map(0, 100));
  ASSERT_TRUE(mapper.map(10, 100));
  EXPECT_FALSE(mapper.map(10, 110));
  EXPECT_FALSE(mapper.map(9, 120));
  ASSERT_TRUE(mapper.map(11, 130));
}

TEST(RelativeRosTimeMapper, RejectsMappedTimestampOverflow)
{
  RelativeRosTimeMapper mapper;
  ASSERT_TRUE(mapper.map(1, std::numeric_limits<std::int64_t>::max() - 1));
  EXPECT_FALSE(mapper.map(3, std::numeric_limits<std::int64_t>::max()));
}

TEST(RelativeRosTimeMapper, ResetStartsANewClockMapping)
{
  RelativeRosTimeMapper mapper;
  ASSERT_TRUE(mapper.map(10, 100));
  mapper.reset();
  const auto mapped = mapper.map(1, 1000);
  ASSERT_TRUE(mapped);
  EXPECT_EQ(*mapped, 1000);
}

}  // namespace ugv_localization_mvp
