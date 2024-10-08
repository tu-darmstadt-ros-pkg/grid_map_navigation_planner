//=================================================================================================
// Copyright (c) 2016, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <grid_map_planner_lib/grid_map_planner.h>
#include <grid_map_planner_lib/grid_map_utils.h>

#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_proc/grid_map_transforms.h>
#include <grid_map_proc/grid_map_path_planning.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace grid_map_planner;

  GridMapPlanner::GridMapPlanner()
  : lethal_dist_(4.0)
  , penalty_dist_(12.0)
  , penalty_weight_(1.0)
  , goal_dist_from_obstacles_(0.3)
  {}
  
  
  GridMapPlanner::~GridMapPlanner()
  {}

  bool GridMapPlanner::setMap(const grid_map::GridMap& map)
  {
    if (!map.exists("occupancy")){
      ROS_ERROR("Tried to set map for grid map planner, but has no occupancy layer!");
      return false;
    }

   this->planning_map_ = map;
   return true;
   /*
   ros::WallTime start_time = ros::WallTime::now();

   if (!grid_map_transforms::addDistanceTransformCv(this->planning_map_)){
     ROS_WARN("Unable to generate distance transform!");
   }

   ROS_INFO_STREAM("Distance transform took " << (ros::WallTime::now() - start_time).toSec() * 1000 << " ms\n");


   std::vector<grid_map::Index> goals;

   goals.push_back(grid_map::Index(200, 310));

   start_time = ros::WallTime::now();

   if (!grid_map_transforms::addExplorationTransform(this->planning_map_, goals)){
     ROS_WARN("Unable to generate distance transform!");
   }

   ROS_INFO_STREAM ("Exploration transform took " << (ros::WallTime::now() - start_time).toSec() * 1000 << " ms\n");



   //std::cout << "layers: " << this->planning_map_.getLayers().size() << "\n";
   */
  }

  bool GridMapPlanner::makeExplorationPlan(const geometry_msgs::Pose &start,std::vector<geometry_msgs::PoseStamped> &plan)
  {
    if (!this->planning_map_.exists("occupancy")){
      ROS_ERROR("Tried to generate exploration plan, but map has no occupancy layer!");
      return false;
    }


    grid_map::Index start_index;

    if (!this->planning_map_.getIndex(grid_map::Position(start.position.x, start.position.y),
                                      start_index))
    {
      ROS_WARN("Goal coords outside map, unable to plan!");
      return false;
    }

    if (!grid_map_transforms::addDistanceTransform(this->planning_map_, start_index, obstacle_cells_, frontier_cells_,
                                                   min_frontier_dist_, min_frontier_size_))
    {
      ROS_WARN("Failed to compute distance transform!");
      return false;
    }
    if (frontier_cells_.empty()) {
      ROS_INFO_STREAM("Did not find any frontiers, map is completely explored.");
      plan.clear();
      return true;
    }

    if (!grid_map_transforms::addExplorationTransform(this->planning_map_,
        frontier_cells_,
        lethal_dist_,
        penalty_dist_))
    {
      ROS_WARN("Unable to generate exploration transform!");
      return false;
    }

    geometry_msgs::Pose adjusted_start;

    grid_map_path_planning::adjustStartPoseIfOccupied(this->planning_map_,
                              start,
                              adjusted_start);

    if(!grid_map_path_planning::findPathExplorationTransform(this->planning_map_,
                                                         adjusted_start,
                                                         plan)){
      ROS_WARN("Find path on exploration transform failed!");
      return false;
    }

    return true;
  }

  bool GridMapPlanner::makePlan(const geometry_msgs::Pose &start,
                                const geometry_msgs::Pose &original_goal,
                                std::vector<geometry_msgs::PoseStamped> &plan,
                                float* plan_cost)
  {
    if (!this->planning_map_.exists("occupancy")){
      ROS_ERROR("Tried to generate plan to goal, but map has no occupancy layer!");
      return false;
    }

    grid_map::Index start_index;

    if (!this->planning_map_.getIndex(grid_map::Position(start.position.x, start.position.y),
                                      start_index))
    {
      ROS_WARN("Start coords [%f, %f, %f] outside map, unable to plan!", start.position.x, start.position.y, start.position.z);
      return false;
    }

    // If there's no distance transform layer, map is new (and we assume if it comes with one, it should be valid)
    // It can also happen that start seed is in different area in map, then also recompute distance transform.
    if (!this->planning_map_.exists("distance_transform") ||
        (this->planning_map_.at("distance_transform", start_index) == std::numeric_limits<float>::max() )){
      if (!grid_map_transforms::addDistanceTransform(this->planning_map_, start_index, obstacle_cells_, frontier_cells_,
                                                     min_frontier_dist_, min_frontier_size_))
      {
        ROS_WARN_STREAM("Failed computing distance transform!" << start.position.x << start.position.y);
        return false;
      }
    }else{
      ROS_DEBUG_STREAM("Using cached distance transform");
    }


    std::vector<grid_map::Index> goals;

    grid_map::Index goal_index;

    if (!this->planning_map_.getIndex(grid_map::Position(original_goal.position.x, original_goal.position.y),
                                      goal_index))
    {
      ROS_WARN("Original goal coords %f, %f outside map, unable to plan!",original_goal.position.x, original_goal.position.y);
      return false;
    }

    // Adjust goal pose and try to move it farther away from walls if possible
    grid_map::Index goal_index_adjusted;
    float required_cell_distance = static_cast<float>(goal_dist_from_obstacles_) / planning_map_.getResolution();
    if (grid_map_path_planning::findValidClosePoseExplorationTransform(this->planning_map_, goal_index, goal_index_adjusted, 3.0, required_cell_distance, required_cell_distance))
    {
       ROS_INFO("Moved goal");
       goal_index = goal_index_adjusted;
    }

    goals.push_back(goal_index);

    if (!grid_map_transforms::addExplorationTransform(this->planning_map_,
        goals,
        lethal_dist_,
        penalty_dist_,
        penalty_weight_))
    {
      ROS_WARN("Unable to generate exploration transform!");
      return false;
    }

    geometry_msgs::Pose adjusted_start;

    grid_map_path_planning::adjustStartPoseIfOccupied(this->planning_map_,
                              start,
                              adjusted_start);

    if(!grid_map_path_planning::findPathExplorationTransform(this->planning_map_,
                                                         adjusted_start,
                                                         plan,
                                                         plan_cost)){
      ROS_WARN("Find path on exploration transform failed!");
      return false;
    }
    
    if (plan.size() == 0){
      ROS_INFO_STREAM("Couldn't find path, returning empty one.");
      return false;
    }

    plan.back().pose.orientation = original_goal.orientation;

    return true;
  }

  bool GridMapPlanner::makePlan(const geometry_msgs::Pose &start,
                                const std::vector<boost::shared_ptr<grid_map_planner_goal_types::MapGoalBase> >& map_goals,
                                std::vector<geometry_msgs::PoseStamped> &plan,
                                int& reached_goal_idx,
                                float* plan_cost)
  {
    if (!this->planning_map_.exists("occupancy")){
      ROS_ERROR("Tried to generate plan to multiple goal poses, but map has no occupancy layer!");
      return false;
    }


    //return false;
    grid_map::Index start_index;

    if (!this->planning_map_.getIndex(grid_map::Position(start.position.x, start.position.y),
                                      start_index))
    {
      ROS_WARN("Start coords outside map, unable to plan!");
      return false;
    }
    if (!grid_map_transforms::addDistanceTransform(this->planning_map_, start_index, obstacle_cells_, frontier_cells_,
                                                   min_frontier_dist_,min_frontier_size_))
    {
      ROS_WARN("Failed computing reachable obstacle cells!");
      return false;
    }


    std::vector<grid_map::Index> goals;
    //grid_map::Index goal_index;

    for (size_t i = 0; i < map_goals.size(); ++i)
    {
      map_goals[i]->getGoalIndices(goals);
    }

    //std::cout << "map_goals size " << map_goals.size() << " goals size: " << goals.size() << "\n";

    if (!grid_map_transforms::addExplorationTransform(this->planning_map_,
        goals,
        lethal_dist_,
        penalty_dist_,
        penalty_weight_))
    {
      ROS_WARN("Unable to generate exploration transform!");
      return false;
    }
    
    geometry_msgs::Pose adjusted_start;

    grid_map_path_planning::adjustStartPoseIfOccupied(this->planning_map_,
                              start,
                              adjusted_start);

    if(!grid_map_path_planning::findPathExplorationTransform(this->planning_map_,
                                                         adjusted_start,
                                                         plan,
                                                         plan_cost)){
      ROS_WARN("Find path on exploration transform failed!");
      return false;
    }
    
    if (plan.size() == 0){
      ROS_INFO_STREAM("Couldn't find path, returning empty one.");
      return false;
    }

    grid_map::Index found_goal_idx;
    this->planning_map_.getIndex(grid_map::Position(plan.back().pose.position.x, plan.back().pose.position.y), found_goal_idx);

    if (map_goals.size() == 1){
      reached_goal_idx = 0;
    }else{

      for (size_t i = 0; i < map_goals.size(); ++i)
      {
        if (map_goals[i]->isReached(found_goal_idx))
        {
          reached_goal_idx = i;
          break;
        }
      }
    }

    plan.back().pose.orientation = map_goals[reached_goal_idx]->getOrientation();

    return true;
  }

bool GridMapPlanner::makePlan(const std::vector<geometry_msgs::Pose>& starts, const geometry_msgs::Pose& original_goal,
                              std::vector<std::vector<geometry_msgs::PoseStamped>>& plans,
                              std::vector<float>* plan_costs)
{
  // prepare output vectors
  plans.resize(starts.size());
  if (plan_costs)
  {
    plan_costs->resize(starts.size(), 0);
  }

  if (!this->planning_map_.exists("occupancy"))
  {
    ROS_ERROR("Tried to generate plan to goal, but map has no occupancy layer!");
    return false;
  }

  grid_map::Index goal_index;

  if (!this->planning_map_.getIndex(grid_map::Position(original_goal.position.x, original_goal.position.y), goal_index))
  {
    ROS_WARN("Original goal coords %f, %f outside map, unable to plan!", original_goal.position.x,
             original_goal.position.y);
    return false;
  }

  for (int i = 0; i < starts.size(); i++)
  {
    geometry_msgs::Pose start = starts[i];

    grid_map::Index start_index;

    if (!this->planning_map_.getIndex(grid_map::Position(start.position.x, start.position.y), start_index))
    {
      ROS_WARN("Start coords [%f, %f, %f] outside map, unable to plan!", start.position.x, start.position.y,
               start.position.z);
      continue;
    }

    // If there's no distance transform layer, map is new (and we assume if it comes with one, it should be valid)
    // It can also happen that start seed is in different area in map, then also recompute distance transform.
    bool computed_distance_transform;
    if (!this->planning_map_.exists("distance_transform") ||
        (this->planning_map_.at("distance_transform", start_index) == std::numeric_limits<float>::max()))
    {
      if (!grid_map_transforms::addDistanceTransform(this->planning_map_, start_index, obstacle_cells_,
                                                     frontier_cells_, min_frontier_dist_))
      {
        ROS_WARN_STREAM("Failed computing distance transform! " << start.position.x << start.position.y);
        continue;
      }
      computed_distance_transform = true;
    }
    else
    {
      ROS_DEBUG_STREAM("Using cached distance transform");
      computed_distance_transform = false;
    }

    // only compute new exploration transform if it does not exist,
    //  a new distance transform was computed or if the existing exploration transform was computed for another goal
    if (!this->planning_map_.exists("exploration_transform") ||
        computed_distance_transform || 
        this->planning_map_.at("exploration_transform", goal_index) != 0)
    {
      // Adjust goal pose and try to move it farther away from walls if possible
      grid_map::Index goal_index_adjusted;
      float required_cell_distance = static_cast<float>(goal_dist_from_obstacles_) / planning_map_.getResolution();
      if (grid_map_path_planning::findValidClosePoseExplorationTransform(
              this->planning_map_, goal_index, goal_index_adjusted, 3.0, required_cell_distance,
              required_cell_distance))
      {
        ROS_INFO("Moved goal");
        goal_index = goal_index_adjusted;
      }

      std::vector<grid_map::Index> goals;
      goals.push_back(goal_index);

      // compute exploration transform for the goal
      if (!grid_map_transforms::addExplorationTransform(this->planning_map_, goals, lethal_dist_, penalty_dist_,
                                                        penalty_weight_))
      {
        ROS_WARN("Unable to generate exploration transform!");
        continue;
      }
    }

    geometry_msgs::Pose adjusted_start;

    grid_map_path_planning::adjustStartPoseIfOccupied(this->planning_map_, start, adjusted_start);

    std::vector<geometry_msgs::PoseStamped> plan;
    float plan_cost;

    // get path from start to goal
    if (!grid_map_path_planning::findPathExplorationTransform(this->planning_map_, adjusted_start, plan, &plan_cost))
    {
      ROS_WARN_STREAM("Find path on exploration transform failed for start (" << adjusted_start.position.x << ", "
                                                                              << adjusted_start.position.y << ", "
                                                                              << adjusted_start.position.z << ")!");
      continue;
    }

    if (plan.size() == 0)
    {
      ROS_WARN_STREAM("Couldn't find path, returning empty one for start (" << adjusted_start.position.x << ", "
                                                                            << adjusted_start.position.y << ", "
                                                                            << adjusted_start.position.z << ").");
      continue;
    }

    plan.back().pose.orientation = original_goal.orientation;

    plans[i] = plan;

    if (plan_costs)
    {
      plan_costs->at(i) = plan_cost;
    }
  }

  return true;
}

bool GridMapPlanner::getValidCloseWaypointForPose(const geometry_msgs::Pose& pose, double min_distance_to_pose,
                                                  double max_distance_to_pose, double min_distance_to_obstacle,
                                                  geometry_msgs::Pose& close_waypoint)
{
  if (!this->planning_map_.exists("occupancy"))
  {
    ROS_ERROR("Tried to get close valid waypoint for pose, but map has no occupancy layer!");
    return false;
  }

  double min_distance_to_obstacle_in_cells = min_distance_to_obstacle / this->planning_map_.getResolution();

  grid_map::Position close_waypoint_gm;

  // compute yaw from pose orientation to get direction of ray cast
  double yaw = grid_map_utils::getYawFromQuaternion(pose.orientation);

  // rotate by 180 degrees to get orientation pointing from the pose to the robot
  // (POIs have the orientation following the line of sight from the robot to the POI)
  yaw = yaw + M_PI;

  double x_start = pose.position.x + min_distance_to_pose * std::cos(yaw);
  double y_start = pose.position.y + min_distance_to_pose * std::sin(yaw);
  grid_map::Position start_position(x_start, y_start);

  double x_end = pose.position.x + max_distance_to_pose * std::cos(yaw);
  double y_end = pose.position.y + max_distance_to_pose * std::sin(yaw);
  grid_map::Position end_position(x_end, y_end);

  if (!this->planning_map_.isInside(start_position))
  {
    ROS_WARN("Point in minimum distance to given pose already outside map! Cannot compute valid close waypoint.");
    return false;
  }

  try
  {
    grid_map::LineIterator line_iterator = grid_map::LineIterator(this->planning_map_, start_position, end_position);

    bool computed_distance_transform = false;
    bool found_valid_point = false;
    for (; !line_iterator.isPastEnd(); ++line_iterator)
    {
      if (this->planning_map_.at("occupancy", *line_iterator) < 50.0)
      {
        if (!computed_distance_transform)
        {
          if (!grid_map_transforms::addDistanceTransform(this->planning_map_, *line_iterator, obstacle_cells_,
                                                         frontier_cells_))
          {
            ROS_WARN("Failed computing reachable obstacle cells!");
            return false;
          }
          computed_distance_transform = true;
        }

        if (this->planning_map_.at("distance_transform", *line_iterator) > min_distance_to_obstacle_in_cells)
        {
          this->planning_map_.getPosition(*line_iterator, close_waypoint_gm);
          found_valid_point = true;
          break;
        }
      }
    }

    if (found_valid_point)
    {
      close_waypoint.position.x = close_waypoint_gm.x();
      close_waypoint.position.y = close_waypoint_gm.y();
      close_waypoint.position.z = 0;

      close_waypoint.orientation =
          grid_map_utils::computePerpendicularOrientation(close_waypoint.position, pose.position);

      return true;
    }

    ROS_WARN("Unable to find valid waypoint on grid map for given start pose!");
    return false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Failed to create line iterator (exception "
                     << e.what() << ")! Start pose: (" << start_position.x() << ", " << start_position.y()
                     << "), end pose: (" << end_position.x() << ", " << end_position.y() << ")");
    return false;
  }
}

bool GridMapPlanner::getValidCloseWaypointForPosition(const geometry_msgs::Point& position,
                                                      double min_distance_to_position, double max_distance_to_position,
                                                      double min_distance_to_obstacle,
                                                      geometry_msgs::Pose& close_waypoint)
{
  grid_map::Index start_index;

  if (!this->planning_map_.getIndex(grid_map::Position(position.x, position.y), start_index))
  {
    ROS_WARN("Start coords outside map, unable to compute close waypoint!");
    return false;
  }

  double min_dist_to_position_in_cells = min_distance_to_position / this->planning_map_.getResolution();
  double max_dist_to_position_in_cells = max_distance_to_position / this->planning_map_.getResolution();
  double min_dist_to_obstacle_in_cells = min_distance_to_obstacle / this->planning_map_.getResolution();

  grid_map::Index close_free_index;
  if (this->planning_map_.at("occupancy", start_index) > 50.0)
  {
    // define criteria for BFS search (find close free cell)
    grid_map_path_planning::StoppingCriteria criteria_free_cell = [](const grid_map::GridMap& grid_map,
                                                                     const grid_map::Index& start_index,
                                                                     const grid_map::Index& current_index) {
      return grid_map.at("occupancy", current_index) < 50.0;
      ;
    };

    // use BFS to find closest free cell
    if (!grid_map_path_planning::findCloseIndexMatchingCriteria(this->planning_map_, start_index, criteria_free_cell,
                                                                close_free_index, max_dist_to_position_in_cells))
    {
      ROS_WARN("Unable to find free close waypoint!");
      return false;
    }
  }
  else
  {
    close_free_index = start_index;
  }

  if (!this->planning_map_.exists("distance_transform"))
  {
    if (!grid_map_transforms::addDistanceTransform(this->planning_map_, close_free_index, obstacle_cells_,
                                                   frontier_cells_))
    {
      ROS_WARN("Failed computing distance transform!");
      return false;
    }
  }

  grid_map::Index waypoint_index;

  // define criteria for BFS search (find close free cell which is min dist to pose and min dist to obstacles)
  grid_map_path_planning::StoppingCriteria criteria =
      [min_dist_to_position_in_cells, min_dist_to_obstacle_in_cells](
          const grid_map::GridMap& grid_map, const grid_map::Index& start_index, const grid_map::Index& current_index) {
        bool is_free = grid_map.at("occupancy", current_index) < 50.0;

        double distance = (Eigen::Vector2d((current_index - start_index).cast<double>())).norm();
        bool is_min_dist_to_pose = distance > min_dist_to_position_in_cells;

        bool is_min_dist_to_obstacle = grid_map.at("distance_transform", current_index) > min_dist_to_obstacle_in_cells;

        return is_free && is_min_dist_to_pose && is_min_dist_to_obstacle;
      };

  // use BFS to find closest free cell
  if (!grid_map_path_planning::findCloseIndexMatchingCriteria(this->planning_map_, close_free_index, criteria,
                                                              waypoint_index, max_dist_to_position_in_cells))
  {
    ROS_WARN("Unable to find free close waypoint!");
    return false;
  }

  grid_map::Position waypoint_position;
  this->planning_map_.getPosition(waypoint_index, waypoint_position);

  close_waypoint.position.x = waypoint_position.x();
  close_waypoint.position.y = waypoint_position.y();
  close_waypoint.position.z = 0;

  close_waypoint.orientation = grid_map_utils::computePerpendicularOrientation(close_waypoint.position, position);

  return true;
}
