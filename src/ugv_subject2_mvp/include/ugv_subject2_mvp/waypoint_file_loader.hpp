#ifndef UGV_SUBJECT2_MVP__WAYPOINT_FILE_LOADER_HPP_
#define UGV_SUBJECT2_MVP__WAYPOINT_FILE_LOADER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ugv_subject2_mvp
{

inline constexpr std::uintmax_t kMaximumWaypointFileBytes = 2U * 1024U * 1024U;
inline constexpr std::size_t kMaximumWaypointLineBytes = 4096U;
inline constexpr std::size_t kMaximumWaypointCount = 10000U;

struct FileWaypoint
{
  double x_m{0.0};
  double y_m{0.0};
  std::optional<double> z_m;
  std::optional<double> yaw_rad;
};

std::vector<FileWaypoint> load_waypoint_csv(const std::string & file_path);

}  // namespace ugv_subject2_mvp

#endif  // UGV_SUBJECT2_MVP__WAYPOINT_FILE_LOADER_HPP_
