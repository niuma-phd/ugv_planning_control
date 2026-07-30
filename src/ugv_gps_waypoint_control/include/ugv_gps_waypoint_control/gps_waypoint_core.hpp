#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ugv_gps_waypoint_control
{

struct GlobalWaypoint
{
  std::int64_t sequence{0};
  double longitude_deg{0.0};
  double latitude_deg{0.0};
  double altitude_m{0.0};
};

struct LocalPoint
{
  double x_east_m{0.0};
  double y_north_m{0.0};
};

struct CourseObservation
{
  double yaw_rad{0.0};
  bool course_updated{false};
};

std::vector<GlobalWaypoint> loadGlobalTrack(const std::string & absolute_path);

class Wgs84EnuProjector
{
public:
  explicit Wgs84EnuProjector(const GlobalWaypoint & origin);
  LocalPoint project(double longitude_deg, double latitude_deg, double altitude_m) const;

private:
  GlobalWaypoint origin_;
  double origin_ecef_x_{0.0};
  double origin_ecef_y_{0.0};
  double origin_ecef_z_{0.0};
};

double cardinalHeadingToEnuYaw(const std::string & heading);

class GpsCourseEstimator
{
public:
  GpsCourseEstimator(double initial_yaw_rad, double minimum_displacement_m);
  bool configIsValid() const noexcept;
  CourseObservation observe(const LocalPoint & position);
  void resetAnchor() noexcept;
  double yaw() const noexcept;

private:
  double yaw_rad_{0.0};
  double minimum_displacement_m_{0.0};
  std::optional<LocalPoint> course_anchor_;
};

}  // namespace ugv_gps_waypoint_control
