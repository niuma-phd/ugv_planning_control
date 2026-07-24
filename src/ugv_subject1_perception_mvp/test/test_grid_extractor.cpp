#include "ugv_subject1_perception_mvp/grid_extractor.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
using ugv_subject1_perception_mvp::GridExtractor;
using ugv_subject1_perception_mvp::GridParameters;
namespace {
GridParameters parameters() {
  GridParameters p;
  p.roi_min_x=0.0; p.roi_max_x=4.0; p.roi_min_y=-2.0; p.roi_max_y=2.0;
  p.min_z=-0.2; p.max_z=1.0; p.self_min_x=0.0; p.self_max_x=0.8;
  p.self_min_y=-0.5; p.self_max_y=0.5; p.cell_size=1.0; p.min_points=2;
  p.corridor_min_x=0.0; p.corridor_max_x=3.0; p.corridor_half_width=0.75;
  return p;
}
}
TEST(GridExtractor, ReportsBodyFrameCellCenterAndCorridorObstacle) {
  GridExtractor extractor(parameters());
  const auto result=extractor.extract({{1.1,0.1,0.2},{1.8,0.2,0.3}});
  ASSERT_EQ(result.occupied_centers.size(),1U);
  EXPECT_DOUBLE_EQ(result.occupied_centers[0].x,1.5);
  EXPECT_DOUBLE_EQ(result.occupied_centers[0].y,0.5);
  EXPECT_TRUE(result.obstacle_detected);
}
TEST(GridExtractor, RemovesVehicleBodyReturnsAndHeightOutliers) {
  GridExtractor extractor(parameters());
  const auto result=extractor.extract({{0.2,0.1,0.2},{0.3,0.2,0.2},{1.2,0.1,-0.3},{1.3,0.2,1.1}});
  EXPECT_TRUE(result.occupied_centers.empty()); EXPECT_FALSE(result.obstacle_detected);
}
TEST(GridExtractor, RequiresMinimumPointsPerCell) {
  GridExtractor extractor(parameters());
  const auto result=extractor.extract({{2.1,1.1,0.2},{2.1,-1.1,0.2}});
  EXPECT_TRUE(result.occupied_centers.empty()); EXPECT_FALSE(result.obstacle_detected);
}
TEST(GridExtractor, IgnoresNonFiniteAndOutOfRoiPoints) {
  GridExtractor extractor(parameters()); const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  const auto result=extractor.extract({{nan,0,0},{1,inf,0},{1,0,nan},{-0.1,1,0},{4.1,1,0},{1,2.1,0}});
  EXPECT_TRUE(result.occupied_centers.empty());
}
TEST(GridExtractor, OccupiedCellOutsideCorridorDoesNotAssertDetection) {
  GridExtractor extractor(parameters()); const auto result=extractor.extract({{2.1,1.1,0.2},{2.2,1.2,0.2}});
  ASSERT_EQ(result.occupied_centers.size(),1U); EXPECT_FALSE(result.obstacle_detected);
}
TEST(GridExtractor, RejectsInvalidConfiguration) {
  auto p=parameters(); p.cell_size=0.0; EXPECT_THROW({GridExtractor extractor(p);},std::invalid_argument);
}
TEST(GridExtractor, RejectsNanCellSize) {
  auto p=parameters(); p.cell_size=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW({GridExtractor extractor(p);},std::invalid_argument);
}
TEST(GridExtractor, RejectsInfiniteCellSize) {
  auto p=parameters(); p.cell_size=std::numeric_limits<double>::infinity();
  EXPECT_THROW({GridExtractor extractor(p);},std::invalid_argument);
}
TEST(GridExtractor, RejectsNanCorridorWidth) {
  auto p=parameters(); p.corridor_half_width=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW({GridExtractor extractor(p);},std::invalid_argument);
}
TEST(GridExtractor, RejectsInfiniteCorridorBound) {
  auto p=parameters(); p.corridor_max_x=std::numeric_limits<double>::infinity();
  EXPECT_THROW({GridExtractor extractor(p);},std::invalid_argument);
}
TEST(GridExtractor, RejectsCellSizeThatOverflowsIntegerGridIndex) {
  auto p=parameters(); p.cell_size=std::numeric_limits<double>::min();
  EXPECT_THROW({GridExtractor extractor(p);},std::invalid_argument);
}
