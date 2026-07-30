#include "ugv_gps_waypoint_control/gps_waypoint_core.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace ugv_gps_waypoint_control
{
namespace
{
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84EccentricitySquared =
  kWgs84Flattening * (2.0 - kWgs84Flattening);
constexpr std::uintmax_t kMaximumFileBytes = 2U * 1024U * 1024U;
constexpr std::size_t kMaximumLineBytes = 4096U;
constexpr std::size_t kMaximumWaypointCount = 10000U;
constexpr double kPi = 3.14159265358979323846;

bool finite(const GlobalWaypoint & point)
{
  return std::isfinite(point.longitude_deg) && std::isfinite(point.latitude_deg) &&
         std::isfinite(point.altitude_m);
}

std::string trim(std::string value)
{
  const auto not_space = [](const unsigned char character) {return !std::isspace(character);};
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::vector<std::string> splitSemicolon(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ';')) {
    fields.push_back(trim(field));
  }
  if (!line.empty() && line.back() == ';') {
    fields.emplace_back();
  }
  return fields;
}

double parseFiniteDouble(
  const std::string & text, const std::string & field, const std::size_t line_number)
{
  std::size_t consumed = 0U;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error(
            "track line " + std::to_string(line_number) + ": invalid " + field);
  }
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::runtime_error(
            "track line " + std::to_string(line_number) + ": invalid " + field);
  }
  return value;
}

std::int64_t parseSequence(const std::string & text, const std::size_t line_number)
{
  std::size_t consumed = 0U;
  long long value = 0;
  try {
    value = std::stoll(text, &consumed, 10);
  } catch (const std::exception &) {
    throw std::runtime_error(
            "track line " + std::to_string(line_number) + ": invalid sequence");
  }
  if (consumed != text.size()) {
    throw std::runtime_error(
            "track line " + std::to_string(line_number) + ": invalid sequence");
  }
  return static_cast<std::int64_t>(value);
}

std::array<double, 3> geodeticToEcef(
  const double longitude_deg, const double latitude_deg, const double altitude_m)
{
  const double longitude = longitude_deg * kPi / 180.0;
  const double latitude = latitude_deg * kPi / 180.0;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double radius = kWgs84SemiMajorAxisM / std::sqrt(
    1.0 - kWgs84EccentricitySquared * sin_latitude * sin_latitude);
  return {
    (radius + altitude_m) * cos_latitude * std::cos(longitude),
    (radius + altitude_m) * cos_latitude * std::sin(longitude),
    (radius * (1.0 - kWgs84EccentricitySquared) + altitude_m) * sin_latitude};
}

std::string normalizedHeading(std::string heading)
{
  heading = trim(std::move(heading));
  std::transform(
    heading.begin(), heading.end(), heading.begin(),
    [](const unsigned char character) {return static_cast<char>(std::toupper(character));});
  return heading;
}
}  // namespace

std::vector<GlobalWaypoint> loadGlobalTrack(const std::string & absolute_path)
{
  if (absolute_path.empty() || !std::filesystem::path(absolute_path).is_absolute()) {
    throw std::invalid_argument("track_file must be a non-empty absolute path");
  }
  const std::filesystem::path path(absolute_path);
  std::error_code error;
  if (!std::filesystem::is_regular_file(path, error) || error) {
    throw std::runtime_error("track_file is not a readable regular file: " + absolute_path);
  }
  const auto size = std::filesystem::file_size(path, error);
  if (error || size > kMaximumFileBytes) {
    throw std::runtime_error("track_file exceeds the 2 MiB limit");
  }
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open track_file: " + absolute_path);
  }

  std::vector<GlobalWaypoint> waypoints;
  std::optional<std::int64_t> previous_sequence;
  std::string line;
  std::size_t line_number = 0U;
  bool header_seen = false;
  while (std::getline(stream, line)) {
    ++line_number;
    if (line.size() > kMaximumLineBytes) {
      throw std::runtime_error("track line exceeds 4096 bytes");
    }
    if (line_number == 1U && line.rfind("\xEF\xBB\xBF", 0U) == 0U) {
      line.erase(0U, 3U);
    }
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    const auto fields = splitSemicolon(line);
    if (!header_seen) {
      const std::vector<std::string> expected{"序号", "经度", "纬度", "高程"};
      if (fields != expected) {
        throw std::runtime_error("track header must be: 序号;经度;纬度;高程");
      }
      header_seen = true;
      continue;
    }
    if (fields.size() != 4U) {
      throw std::runtime_error(
              "track line " + std::to_string(line_number) + " must have four fields");
    }
    GlobalWaypoint waypoint;
    waypoint.sequence = parseSequence(fields[0], line_number);
    waypoint.longitude_deg = parseFiniteDouble(fields[1], "longitude", line_number);
    waypoint.latitude_deg = parseFiniteDouble(fields[2], "latitude", line_number);
    waypoint.altitude_m = parseFiniteDouble(fields[3], "altitude", line_number);
    if (!finite(waypoint) || waypoint.longitude_deg < -180.0 ||
      waypoint.longitude_deg > 180.0 || waypoint.latitude_deg < -90.0 ||
      waypoint.latitude_deg > 90.0)
    {
      throw std::runtime_error(
              "track line " + std::to_string(line_number) + " has invalid WGS84 coordinates");
    }
    if (previous_sequence && waypoint.sequence <= *previous_sequence) {
      throw std::runtime_error("track waypoint sequence must be strictly increasing");
    }
    if (waypoints.size() >= kMaximumWaypointCount) {
      throw std::runtime_error("track_file exceeds the 10000-waypoint limit");
    }
    previous_sequence = waypoint.sequence;
    waypoints.push_back(waypoint);
  }
  if (!stream.eof()) {
    throw std::runtime_error("failed while reading track_file");
  }
  if (!header_seen || waypoints.size() < 2U) {
    throw std::runtime_error("track_file must contain a header and at least two waypoints");
  }

  Wgs84EnuProjector projector(waypoints.front());
  const auto first = projector.project(
    waypoints.front().longitude_deg, waypoints.front().latitude_deg,
    waypoints.front().altitude_m);
  bool nonzero_segment = false;
  for (std::size_t index = 1U; index < waypoints.size(); ++index) {
    const auto point = projector.project(
      waypoints[index].longitude_deg, waypoints[index].latitude_deg,
      waypoints[index].altitude_m);
    if (std::hypot(point.x_east_m - first.x_east_m, point.y_north_m - first.y_north_m) >
      1.0e-6)
    {
      nonzero_segment = true;
      break;
    }
  }
  if (!nonzero_segment) {
    throw std::runtime_error("track_file has no non-zero planar segment");
  }
  return waypoints;
}

Wgs84EnuProjector::Wgs84EnuProjector(const GlobalWaypoint & origin)
: origin_(origin)
{
  if (!finite(origin_) || origin_.longitude_deg < -180.0 || origin_.longitude_deg > 180.0 ||
    origin_.latitude_deg < -90.0 || origin_.latitude_deg > 90.0)
  {
    throw std::invalid_argument("WGS84 projection origin is invalid");
  }
  const auto ecef = geodeticToEcef(
    origin_.longitude_deg, origin_.latitude_deg, origin_.altitude_m);
  origin_ecef_x_ = ecef[0];
  origin_ecef_y_ = ecef[1];
  origin_ecef_z_ = ecef[2];
}

LocalPoint Wgs84EnuProjector::project(
  const double longitude_deg, const double latitude_deg, const double altitude_m) const
{
  const GlobalWaypoint point{0, longitude_deg, latitude_deg, altitude_m};
  if (!finite(point) || longitude_deg < -180.0 || longitude_deg > 180.0 ||
    latitude_deg < -90.0 || latitude_deg > 90.0)
  {
    throw std::invalid_argument("WGS84 point is invalid");
  }
  const auto ecef = geodeticToEcef(longitude_deg, latitude_deg, altitude_m);
  const double dx = ecef[0] - origin_ecef_x_;
  const double dy = ecef[1] - origin_ecef_y_;
  const double dz = ecef[2] - origin_ecef_z_;
  const double longitude = origin_.longitude_deg * kPi / 180.0;
  const double latitude = origin_.latitude_deg * kPi / 180.0;
  const double east = -std::sin(longitude) * dx + std::cos(longitude) * dy;
  const double north =
    -std::sin(latitude) * std::cos(longitude) * dx -
    std::sin(latitude) * std::sin(longitude) * dy + std::cos(latitude) * dz;
  return LocalPoint{east, north};
}

double cardinalHeadingToEnuYaw(const std::string & heading)
{
  const auto value = normalizedHeading(heading);
  if (value == "E" || value == "EAST" || value == "东" || value == "正东") {
    return 0.0;
  }
  if (value == "N" || value == "NORTH" || value == "北" || value == "正北") {
    return kPi / 2.0;
  }
  if (value == "W" || value == "WEST" || value == "西" || value == "正西") {
    return kPi;
  }
  if (value == "S" || value == "SOUTH" || value == "南" || value == "正南") {
    return -kPi / 2.0;
  }
  throw std::invalid_argument(
          "initial_heading must be EAST, SOUTH, WEST, NORTH, or the matching Chinese cardinal");
}

GpsCourseEstimator::GpsCourseEstimator(
  const double initial_yaw_rad, const double minimum_displacement_m)
: yaw_rad_(std::atan2(std::sin(initial_yaw_rad), std::cos(initial_yaw_rad))),
  minimum_displacement_m_(minimum_displacement_m)
{
}

bool GpsCourseEstimator::configIsValid() const noexcept
{
  return std::isfinite(yaw_rad_) && std::isfinite(minimum_displacement_m_) &&
         minimum_displacement_m_ > 0.0;
}

CourseObservation GpsCourseEstimator::observe(const LocalPoint & position)
{
  CourseObservation result{yaw_rad_, false};
  if (!configIsValid() || !std::isfinite(position.x_east_m) ||
    !std::isfinite(position.y_north_m))
  {
    return result;
  }
  if (!course_anchor_) {
    course_anchor_ = position;
    return result;
  }
  const double dx = position.x_east_m - course_anchor_->x_east_m;
  const double dy = position.y_north_m - course_anchor_->y_north_m;
  if (std::hypot(dx, dy) < minimum_displacement_m_) {
    return result;
  }
  yaw_rad_ = std::atan2(dy, dx);
  course_anchor_ = position;
  result.yaw_rad = yaw_rad_;
  result.course_updated = true;
  return result;
}

void GpsCourseEstimator::resetAnchor() noexcept
{
  course_anchor_.reset();
}

double GpsCourseEstimator::yaw() const noexcept
{
  return yaw_rad_;
}

}  // namespace ugv_gps_waypoint_control
