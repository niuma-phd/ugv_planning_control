#include "ugv_subject1_perception_mvp/grid_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

namespace ugv_subject1_perception_mvp
{

GridExtractor::GridExtractor(GridParameters parameters)
: parameters_(parameters)
{
  const double maximum_grid_index = std::max(
    {std::abs(parameters_.roi_min_x), std::abs(parameters_.roi_max_x),
      std::abs(parameters_.roi_min_y), std::abs(parameters_.roi_max_y)}) /
    parameters_.cell_size;
  const bool finite_parameters =
    std::isfinite(parameters_.roi_min_x) &&
    std::isfinite(parameters_.roi_max_x) &&
    std::isfinite(parameters_.roi_min_y) &&
    std::isfinite(parameters_.roi_max_y) &&
    std::isfinite(parameters_.min_z) &&
    std::isfinite(parameters_.max_z) &&
    std::isfinite(parameters_.self_min_x) &&
    std::isfinite(parameters_.self_max_x) &&
    std::isfinite(parameters_.self_min_y) &&
    std::isfinite(parameters_.self_max_y) &&
    std::isfinite(parameters_.cell_size) &&
    std::isfinite(parameters_.corridor_min_x) &&
    std::isfinite(parameters_.corridor_max_x) &&
    std::isfinite(parameters_.corridor_half_width);
  if (!finite_parameters ||
    !(parameters_.roi_min_x < parameters_.roi_max_x) ||
    !(parameters_.roi_min_y < parameters_.roi_max_y) ||
    !(parameters_.min_z < parameters_.max_z) || parameters_.cell_size <= 0.0 ||
    parameters_.self_min_x > parameters_.self_max_x ||
    parameters_.self_min_y > parameters_.self_max_y ||
    parameters_.min_points <= 0 || parameters_.corridor_half_width < 0.0 ||
    parameters_.corridor_min_x > parameters_.corridor_max_x ||
    !std::isfinite(maximum_grid_index) ||
    maximum_grid_index >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
  {
    throw std::invalid_argument("invalid grid extractor parameters");
  }
}

GridResult GridExtractor::extract(const std::vector<Point3> & points) const
{
  using Cell = std::pair<std::int64_t, std::int64_t>;
  std::map<Cell, std::size_t> counts;

  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    if (point.x < parameters_.roi_min_x || point.x > parameters_.roi_max_x ||
      point.y < parameters_.roi_min_y || point.y > parameters_.roi_max_y ||
      point.z < parameters_.min_z || point.z > parameters_.max_z)
    {
      continue;
    }
    if (point.x >= parameters_.self_min_x && point.x <= parameters_.self_max_x &&
      point.y >= parameters_.self_min_y && point.y <= parameters_.self_max_y)
    {
      continue;
    }

    const auto ix = static_cast<std::int64_t>(std::floor(point.x / parameters_.cell_size));
    const auto iy = static_cast<std::int64_t>(std::floor(point.y / parameters_.cell_size));
    ++counts[{ix, iy}];
  }

  GridResult result;
  for (const auto & [cell, count] : counts) {
    if (count < static_cast<std::size_t>(parameters_.min_points)) {
      continue;
    }
    Point3 center{
      (static_cast<double>(cell.first) + 0.5) * parameters_.cell_size,
      (static_cast<double>(cell.second) + 0.5) * parameters_.cell_size,
      0.0};
    result.occupied_centers.push_back(center);
    if (center.x >= parameters_.corridor_min_x && center.x <= parameters_.corridor_max_x &&
      std::abs(center.y) <= parameters_.corridor_half_width)
    {
      result.obstacle_detected = true;
    }
  }
  return result;
}

}  // namespace ugv_subject1_perception_mvp
