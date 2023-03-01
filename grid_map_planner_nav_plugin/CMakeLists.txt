cmake_minimum_required(VERSION 3.0.2)
project(grid_map_planner_nav_plugin)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  grid_map_planner_lib
  nav_core
  hector_nav_core
  roscpp
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_map_planner_hector_global_plugin grid_map_planner_global_plugin
  CATKIN_DEPENDS grid_map_planner_lib nav_core hector_nav_core roscpp pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(grid_map_planner_global_plugin  src/grid_map_planner_global_plugin.cpp)
target_link_libraries(grid_map_planner_global_plugin ${catkin_LIBRARIES})

add_library(grid_map_planner_hector_global_plugin  src/grid_map_planner_hector_global_plugin.cpp)
target_link_libraries(grid_map_planner_hector_global_plugin ${catkin_LIBRARIES})
