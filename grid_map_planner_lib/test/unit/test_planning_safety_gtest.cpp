#include <gtest/gtest.h>
#include <grid_map_planner_lib/grid_map_planner.h>
#include <grid_map_proc/grid_map_path_planning.h>
#include <grid_map_proc/grid_map_transforms.h>
#include <cmath>
#include <limits>

namespace {
const float uncomputed = std::numeric_limits<float>::max();

grid_map::GridMap makeMap()
{
  grid_map::GridMap map({"occupancy"});
  map.setGeometry(grid_map::Length(10.0, 10.0), 0.05);
  auto& occupancy = map["occupancy"];
  occupancy.setConstant(0.0f);
  occupancy.row(0).setConstant(100.0f);
  occupancy.row(199).setConstant(100.0f);
  occupancy.col(0).setConstant(100.0f);
  occupancy.col(199).setConstant(100.0f);
  // A wall with a passage around its end, plus a separate thick obstacle.
  occupancy.block(1, 95, 130, 10).setConstant(100.0f);
  occupancy.block(50, 150, 5, 5).setConstant(100.0f);
  return map;
}

geometry_msgs::Pose poseAt(const grid_map::GridMap& map, const grid_map::Index& index)
{
  grid_map::Position position;
  map.getPosition(index, position);
  geometry_msgs::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.orientation.w = 1.0;
  return pose;
}

TEST(DistanceTransformTest, OccupiedSeedDoesNotOverwriteCachedField)
{
  auto map = makeMap();
  std::vector<grid_map::Index> obstacles, frontiers;
  ASSERT_TRUE(grid_map_transforms::addDistanceTransform(map, {40, 40}, obstacles, frontiers));
  const auto before = map["distance_transform"];
  const auto obstacles_before = obstacles;
  const auto frontiers_before = frontiers;
  // This cell's eight neighbours are occupied; one of them borders free space.
  // Seeding here used to replace the field with distances from just eight cells.
  EXPECT_FALSE(grid_map_transforms::addDistanceTransform(map, {51, 152}, obstacles, frontiers));
  EXPECT_TRUE((map["distance_transform"].array() == before.array()).all());
  ASSERT_EQ(obstacles.size(), obstacles_before.size());
  ASSERT_EQ(frontiers.size(), frontiers_before.size());
  for (size_t i = 0; i < obstacles.size(); ++i)
    EXPECT_TRUE((obstacles[i] == obstacles_before[i]).all());
  for (size_t i = 0; i < frontiers.size(); ++i)
    EXPECT_TRUE((frontiers[i] == frontiers_before[i]).all());
}

TEST(DistanceTransformTest, InvalidSeedsDoNotCreatePartialLayers)
{
  const float invalid_occupancies[] = {100.0f, -1.0f, 50.0f,
                                      std::numeric_limits<float>::quiet_NaN()};
  for (float value : invalid_occupancies) {
    auto map = makeMap();
    map.at("occupancy", {40, 40}) = value;
    std::vector<grid_map::Index> obstacles, frontiers;
    EXPECT_FALSE(grid_map_transforms::addDistanceTransform(map, {40, 40}, obstacles, frontiers));
    EXPECT_FALSE(map.exists("distance_transform"));
    EXPECT_FALSE(map.exists("dist_seed_transform"));
  }
}

TEST(DistanceTransformTest, RejectsOutOfBoundsSeeds)
{
  auto map = makeMap();
  std::vector<grid_map::Index> obstacles, frontiers;
  for (const auto& seed : std::vector<grid_map::Index>{{-1, 40}, {40, -1}, {200, 40}, {40, 200}}) {
    EXPECT_FALSE(grid_map_transforms::addDistanceTransform(map, seed, obstacles, frontiers));
    EXPECT_FALSE(grid_map_transforms::collectReachableObstacleCells(map, seed, obstacles, frontiers));
  }
  EXPECT_FALSE(map.exists("distance_transform"));
  EXPECT_FALSE(map.exists("dist_seed_transform"));
}

TEST(ShortcutTest, RejectsOccupiedAndUnknownCellsEvenWithLargeDistances)
{
  auto map = makeMap();
  map.add("distance_transform", 100.0f);
  const grid_map::Index start(40, 40), goal(40, 160);
  EXPECT_FALSE(grid_map_path_planning::shortCutValid(map, map["distance_transform"], start, goal));
  map["occupancy"].setZero();
  for (float value : {100.0f, -1.0f, std::numeric_limits<float>::quiet_NaN()}) {
    map.at("occupancy", {40, 80}) = value;
    EXPECT_FALSE(grid_map_path_planning::shortCutValid(map, map["distance_transform"], start, goal));
  }
}

TEST(ShortcutTest, RejectsUncomputedAndNonfiniteDistances)
{
  auto map = makeMap();
  map["occupancy"].setZero();
  for (float value : {uncomputed, std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::quiet_NaN()}) {
    map.add("distance_transform", 100.0f);
    map.at("distance_transform", {40, 80}) = value;
    EXPECT_FALSE(grid_map_path_planning::shortCutValid(map, map["distance_transform"], {40, 40}, {40, 160}));
  }
}

TEST(ShortcutTest, EndpointExceptionOnlyRelaxesClearance)
{
  auto map = makeMap();
  map["occupancy"].setZero();
  map.add("distance_transform", 100.0f);
  map.at("distance_transform", {40, 160}) = 5.0f;
  EXPECT_TRUE(grid_map_path_planning::shortCutValid(map, map["distance_transform"], {40, 40}, {40, 160}));
  map.at("occupancy", {40, 160}) = 100.0f;
  EXPECT_FALSE(grid_map_path_planning::shortCutValid(map, map["distance_transform"], {40, 40}, {40, 160}));
}

TEST(GridMapPlannerTest, FailedRequestsDoNotPoisonSubsequentRoutes)
{
  const auto map = makeMap();
  grid_map_planner::GridMapPlanner planner;
  ASSERT_TRUE(planner.setMap(map));
  const auto start = poseAt(map, {40, 40});
  const auto goal = poseAt(map, {40, 160});
  const auto invalid = poseAt(map, {51, 152});
  std::vector<geometry_msgs::PoseStamped> before, failed, after;
  ASSERT_TRUE(planner.makePlan(start, goal, before));
  ASSERT_GT(before.size(), 3u);
  EXPECT_FALSE(planner.makePlan(start, invalid, failed));
  EXPECT_FALSE(planner.makePlan(invalid, goal, failed));
  ASSERT_TRUE(planner.makePlan(start, goal, after));
  ASSERT_EQ(before.size(), after.size());
  for (size_t i = 0; i < before.size(); ++i) {
    EXPECT_DOUBLE_EQ(before[i].pose.position.x, after[i].pose.position.x);
    EXPECT_DOUBLE_EQ(before[i].pose.position.y, after[i].pose.position.y);
  }
}
TEST(ShortcutTest, HonorsCustomLayersDuringPathExtraction)
{
  auto map = makeMap();
  map.add("custom_occupancy", map["occupancy"]);
  map.erase("occupancy");
  std::vector<grid_map::Index> obstacles, frontiers;
  ASSERT_TRUE(grid_map_transforms::addDistanceTransform(
      map, {40, 40}, obstacles, frontiers, 1.0f, 1, "custom_occupancy", "custom_distances"));
  ASSERT_TRUE(grid_map_transforms::addExplorationTransform(
      map, {{40, 160}}, 4.0f, 12.0f, 1.0f, "custom_occupancy", "custom_distances", "custom_costs"));
  std::vector<geometry_msgs::PoseStamped> path;
  ASSERT_TRUE(grid_map_path_planning::findPathExplorationTransform(
      map, poseAt(map, {40, 40}), path, nullptr, "custom_occupancy", "custom_distances", "custom_costs"));
  ASSERT_GT(path.size(), 3u);
  for (size_t i = 1; i < path.size(); ++i) {
    grid_map::Index from, to;
    ASSERT_TRUE(map.getIndex({path[i-1].pose.position.x, path[i-1].pose.position.y}, from));
    ASSERT_TRUE(map.getIndex({path[i].pose.position.x, path[i].pose.position.y}, to));
    for (grid_map::LineIterator it(map, from, to); !it.isPastEnd(); ++it)
      ASSERT_FLOAT_EQ(map.at("custom_occupancy", *it), 0.0f);
  }
}

TEST(GridMapPlannerTest, EntirelyFreeMapStillProducesAPath)
{
  auto map = makeMap();
  map["occupancy"].setZero();
  grid_map_planner::GridMapPlanner planner;
  ASSERT_TRUE(planner.setMap(map));
  std::vector<geometry_msgs::PoseStamped> path;
  ASSERT_TRUE(planner.makePlan(poseAt(map, {40, 40}), poseAt(map, {60, 60}), path));
  ASSERT_GE(path.size(), 2u);
  EXPECT_DOUBLE_EQ(path.front().pose.position.x, poseAt(map, {40, 40}).position.x);
  EXPECT_DOUBLE_EQ(path.back().pose.position.y, poseAt(map, {60, 60}).position.y);
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
