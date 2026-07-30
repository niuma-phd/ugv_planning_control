#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "ugv_gps_waypoint_control/gps_waypoint_core.hpp"

namespace
{
using ugv_gps_waypoint_control::GlobalWaypoint;
using ugv_gps_waypoint_control::GpsCourseEstimator;
using ugv_gps_waypoint_control::LocalPoint;
using ugv_gps_waypoint_control::Wgs84EnuProjector;
using ugv_gps_waypoint_control::cardinalHeadingToEnuYaw;
using ugv_gps_waypoint_control::loadGlobalTrack;

constexpr double kPi = 3.14159265358979323846;

class TemporaryTrack
{
public:
  explicit TemporaryTrack(const std::string & content)
  {
    char pattern[] = "/tmp/gps_track_XXXXXX";
    const int descriptor = mkstemp(pattern);
    if (descriptor < 0) {
      throw std::runtime_error("mkstemp failed");
    }
    close(descriptor);
    path_ = pattern;
    std::ofstream stream(path_);
    stream << content;
  }
  ~TemporaryTrack() {std::remove(path_.c_str());}
  const std::string & path() const {return path_;}

private:
  std::string path_;
};

TEST(GlobalTrack, LoadsUtf8HeaderAndStrictSequence)
{
  TemporaryTrack file(
    "序号;经度;纬度;高程\n"
    "1;118.81295711;32.09341467;0\n"
    "2;118.81321460;32.09337378;0\n");
  const auto points = loadGlobalTrack(file.path());
  ASSERT_EQ(points.size(), 2U);
  EXPECT_EQ(points[0].sequence, 1);
  EXPECT_DOUBLE_EQ(points[1].longitude_deg, 118.81321460);
}

TEST(GlobalTrack, RejectsWrongHeaderAndNonIncreasingSequence)
{
  TemporaryTrack wrong_header(
    "index;lon;lat;height\n1;118;32;0\n2;118.1;32;0\n");
  EXPECT_THROW(loadGlobalTrack(wrong_header.path()), std::runtime_error);

  TemporaryTrack duplicate(
    "序号;经度;纬度;高程\n1;118;32;0\n1;118.1;32;0\n");
  EXPECT_THROW(loadGlobalTrack(duplicate.path()), std::runtime_error);
}

TEST(Wgs84Projection, EastAndNorthUseStandardEnuAxes)
{
  const GlobalWaypoint origin{1, 118.0, 32.0, 0.0};
  Wgs84EnuProjector projector(origin);
  const auto east = projector.project(118.001, 32.0, 0.0);
  const auto north = projector.project(118.0, 32.001, 0.0);
  EXPECT_GT(east.x_east_m, 90.0);
  EXPECT_NEAR(east.y_north_m, 0.0, 0.01);
  EXPECT_GT(north.y_north_m, 110.0);
  EXPECT_NEAR(north.x_east_m, 0.0, 0.01);
}

TEST(CardinalHeading, MapsToEnuYaw)
{
  EXPECT_DOUBLE_EQ(cardinalHeadingToEnuYaw("EAST"), 0.0);
  EXPECT_DOUBLE_EQ(cardinalHeadingToEnuYaw("正东"), 0.0);
  EXPECT_DOUBLE_EQ(cardinalHeadingToEnuYaw("NORTH"), kPi / 2.0);
  EXPECT_DOUBLE_EQ(cardinalHeadingToEnuYaw("WEST"), kPi);
  EXPECT_DOUBLE_EQ(cardinalHeadingToEnuYaw("SOUTH"), -kPi / 2.0);
  EXPECT_THROW(cardinalHeadingToEnuYaw("NORTHEAST"), std::invalid_argument);
}

TEST(GpsCourseEstimator, KeepsInitialYawUntilExplicitBaselineIsCrossed)
{
  GpsCourseEstimator estimator(kPi / 2.0, 5.0);
  EXPECT_TRUE(estimator.configIsValid());
  EXPECT_FALSE(estimator.observe(LocalPoint{0.0, 0.0}).course_updated);
  const auto short_move = estimator.observe(LocalPoint{4.9, 0.0});
  EXPECT_FALSE(short_move.course_updated);
  EXPECT_DOUBLE_EQ(short_move.yaw_rad, kPi / 2.0);
  const auto long_move = estimator.observe(LocalPoint{5.0, 0.0});
  EXPECT_TRUE(long_move.course_updated);
  EXPECT_NEAR(long_move.yaw_rad, 0.0, 1.0e-12);
}

TEST(GpsCourseEstimator, ResetDropsOnlyThePositionAnchor)
{
  GpsCourseEstimator estimator(kPi / 2.0, 5.0);
  estimator.observe(LocalPoint{0.0, 0.0});
  EXPECT_TRUE(estimator.observe(LocalPoint{5.0, 0.0}).course_updated);
  estimator.resetAnchor();
  const auto first_after_reset = estimator.observe(LocalPoint{100.0, 100.0});
  EXPECT_FALSE(first_after_reset.course_updated);
  EXPECT_NEAR(first_after_reset.yaw_rad, 0.0, 1.0e-12);
}
}  // namespace
