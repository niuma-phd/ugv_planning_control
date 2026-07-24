#pragma once

#include <cstddef>
#include <vector>

namespace ugv_subject1_perception_mvp
{

struct Point3
{
  double x;
  double y;
  double z;
};

struct GridParameters
{
  double roi_min_x{0.0};
  double roi_max_x{8.0};
  double roi_min_y{-3.0};
  double roi_max_y{3.0};
  double min_z{-0.4};
  double max_z{1.5};
  double self_min_x{-1.0};
  double self_max_x{1.0};
  double self_min_y{-0.8};
  double self_max_y{0.8};
  double cell_size{0.25};
  int min_points{3};
  double corridor_min_x{0.0};
  double corridor_max_x{5.0};
  double corridor_half_width{1.0};
};

struct GridResult
{
  std::vector<Point3> occupied_centers;
  bool obstacle_detected{false};
};

class GridExtractor
{
public:
  explicit GridExtractor(GridParameters parameters);
  GridResult extract(const std::vector<Point3> & points) const;

private:
  GridParameters parameters_;
};

}  // namespace ugv_subject1_perception_mvp
