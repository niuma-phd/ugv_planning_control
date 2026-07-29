#include "ugv_subject2_mvp/waypoint_file_loader.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <unistd.h>

#include "gtest/gtest.h"

namespace
{

class TemporaryCsv
{
public:
  explicit TemporaryCsv(const std::string & contents)
  {
    char path[] = "/tmp/ugv_waypoints_XXXXXX";
    const int descriptor = mkstemp(path);
    if (descriptor < 0) {
      throw std::runtime_error("mkstemp failed");
    }
    close(descriptor);
    path_ = path;
    std::ofstream output(path_);
    output << contents;
  }

  ~TemporaryCsv() {std::remove(path_.c_str());}
  const std::string & path() const {return path_;}

private:
  std::string path_;
};

}  // namespace

TEST(WaypointFileLoader, LoadsArbitraryColumnOrderAndOptionalValues)
{
  const TemporaryCsv file(
    "# map-frame route\n\n yaw_rad, y_m, x_m, z_m\n"
    "0.1, 2.0, 1.0, 3.0\n0.2,4.0,5.0,6.0\n");
  const auto points = ugv_subject2_mvp::load_waypoint_csv(file.path());
  ASSERT_EQ(points.size(), 2U);
  EXPECT_DOUBLE_EQ(points[0].x_m, 1.0);
  EXPECT_DOUBLE_EQ(points[0].y_m, 2.0);
  ASSERT_TRUE(points[0].z_m.has_value());
  EXPECT_DOUBLE_EQ(*points[0].z_m, 3.0);
  ASSERT_TRUE(points[0].yaw_rad.has_value());
  EXPECT_DOUBLE_EQ(*points[0].yaw_rad, 0.1);
}

TEST(WaypointFileLoader, LoadsOnlyRequiredColumns)
{
  const TemporaryCsv file("x_m,y_m\n0,0\n1,0\n");
  const auto points = ugv_subject2_mvp::load_waypoint_csv(file.path());
  ASSERT_EQ(points.size(), 2U);
  EXPECT_FALSE(points[0].z_m.has_value());
  EXPECT_FALSE(points[0].yaw_rad.has_value());
}

TEST(WaypointFileLoader, LoadsAbsolutePathContainingSpacesAndChineseCharacters)
{
  const std::string path =
    "/tmp/ugv waypoint 航点 " + std::to_string(static_cast<long long>(getpid())) + ".csv";
  {
    std::ofstream output(path);
    ASSERT_TRUE(output.is_open());
    output << "y_m,x_m\n0,0\n1,2\n";
  }
  const auto points = ugv_subject2_mvp::load_waypoint_csv(path);
  std::remove(path.c_str());
  ASSERT_EQ(points.size(), 2U);
  EXPECT_DOUBLE_EQ(points[1].x_m, 2.0);
  EXPECT_DOUBLE_EQ(points[1].y_m, 1.0);
}

TEST(WaypointFileLoader, RejectsMissingRequiredDuplicateAndUnknownColumns)
{
  const TemporaryCsv missing("x_m,z_m\n0,0\n1,0\n");
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv(missing.path()), std::runtime_error);
  const TemporaryCsv duplicate("x_m,y_m,x_m\n0,0,0\n1,1,1\n");
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv(duplicate.path()), std::runtime_error);
  const TemporaryCsv unknown("x_m,y_m,name\n0,0,a\n1,1,b\n");
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv(unknown.path()), std::runtime_error);
}

TEST(WaypointFileLoader, RejectsMalformedOrNonFiniteValues)
{
  const TemporaryCsv malformed("x_m,y_m\n0,0\n1x,2\n");
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv(malformed.path()), std::runtime_error);
  const TemporaryCsv nonfinite("x_m,y_m\n0,0\nnan,2\n");
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv(nonfinite.path()), std::runtime_error);
  const TemporaryCsv empty_optional("x_m,y_m,z_m\n0,0,\n1,1,2\n");
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv(empty_optional.path()), std::runtime_error);
}

TEST(WaypointFileLoader, RejectsTooFewPointsAndCoincidentRoute)
{
  const TemporaryCsv too_few("x_m,y_m\n0,0\n");
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv(too_few.path()), std::runtime_error);
  const TemporaryCsv coincident("x_m,y_m\n1,2\n1,2\n1,2\n");
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv(coincident.path()), std::runtime_error);
}

TEST(WaypointFileLoader, RejectsMissingAndEmptyFiles)
{
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv(""), std::invalid_argument);
  EXPECT_THROW(
    ugv_subject2_mvp::load_waypoint_csv("relative/waypoints.csv"),
    std::invalid_argument);
  EXPECT_THROW(
    ugv_subject2_mvp::load_waypoint_csv("/definitely/not/a/waypoint/file.csv"),
    std::runtime_error);
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv("/tmp"), std::runtime_error);
  const TemporaryCsv empty("# only comments\n\n");
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv(empty.path()), std::runtime_error);
}

TEST(WaypointFileLoader, RejectsFileLargerThanTwoMebibytes)
{
  const TemporaryCsv oversized(
    std::string(ugv_subject2_mvp::kMaximumWaypointFileBytes + 1U, 'x'));
  EXPECT_THROW(ugv_subject2_mvp::load_waypoint_csv(oversized.path()), std::runtime_error);
}

TEST(WaypointFileLoader, RejectsLineLongerThan4096Bytes)
{
  const TemporaryCsv oversized_line(
    "x_m,y_m\n" +
    std::string(ugv_subject2_mvp::kMaximumWaypointLineBytes + 1U, '1') +
    ",0\n1,1\n");
  EXPECT_THROW(
    ugv_subject2_mvp::load_waypoint_csv(oversized_line.path()), std::runtime_error);
}

TEST(WaypointFileLoader, RejectsMoreThan10000Waypoints)
{
  std::ostringstream contents;
  contents << "x_m,y_m\n";
  for (std::size_t index = 0U;
    index < ugv_subject2_mvp::kMaximumWaypointCount + 1U; ++index)
  {
    contents << index << ",0\n";
  }
  const TemporaryCsv excessive_points(contents.str());
  EXPECT_THROW(
    ugv_subject2_mvp::load_waypoint_csv(excessive_points.path()), std::runtime_error);
}
