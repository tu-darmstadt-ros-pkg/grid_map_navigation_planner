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

    if (!grid_map_transforms::addDistanceTransform(this->planning_map_, start_index, obstacle_cells_, frontier_cells_))
    {
      ROS_WARN("Failed to compute distance transform!");
      return false;
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
      ROS_WARN("Start coords %f outside map, unable to plan!",start);
      return false;
    }

    if (!grid_map_transforms::addDistanceTransform(this->planning_map_, start_index, obstacle_cells_, frontier_cells_))
    {
      ROS_WARN_STREAM("Failed computing reachable obstacle cells!" << start.position.x << start.position.y);
      return false;
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
    if (grid_map_path_planning::findValidClosePoseExplorationTransform(this->planning_map_, goal_index, goal_index_adjusted))
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

    if (!grid_map_transforms::addDistanceTransform(this->planning_map_, start_index, obstacle_cells_, frontier_cells_))
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
