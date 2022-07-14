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

#ifndef GRID_MAP_PLANNER_H___
#define GRID_MAP_PLANNER_H___

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

// Grid Map
#include <grid_map_core/GridMap.hpp>

#include <grid_map_planner_lib/grid_map_planner_goal_types.h>

//#include <dynamic_reconfigure/server.h>


//#include <boost/shared_array.hpp>

namespace grid_map_planner{

//class ExplorationPlannerConfig;

class GridMapPlanner {
public:
  GridMapPlanner();
  ~GridMapPlanner();

  /**
   * Plans from start to given goal.
   * @param start The start point
   * @param goal The goal point
   * @param plan The generated plan
   * @param plan_cost The plan cost for the returned plan
   */
  bool makePlan(const geometry_msgs::Pose &start,
                const geometry_msgs::Pose &original_goal,
                std::vector<geometry_msgs::PoseStamped> &plan,
                float* plan_cost = 0);

  /**
   * Plans from start the goal point with the least cost to reach.
   * If the found goal point is part of multiple map_goals, only the first
   * one found is returned in reached_goal_idx and it's orientation used.
   * @param start The start point
   * @param map_goals A vector of allowed goals
   * @param plan The generated plan
   * @param reached_goal_idx The index the reached goal belongs to in map_goals
   * @param plan_cost The plan cost for the returned plan
   */
  bool makePlan(const geometry_msgs::Pose &start,
                const std::vector<boost::shared_ptr<grid_map_planner_goal_types::MapGoalBase> >& map_goals,
                std::vector<geometry_msgs::PoseStamped> &plan,
                int& reached_goal_idx,
                float* plan_cost = 0);

  /**
    * Given a start point, finds a frontier between known and unknown space and generates a plan to go there. If the map
    * is fully discovered, an empty path will be returned
    * @param start The start point
    * @param plan The plan to explore into unknown space or empty plan
    * @return Returns true, if planning was successful or the map is fully discovered
    */
  bool makeExplorationPlan(const geometry_msgs::Pose &start,std::vector<geometry_msgs::PoseStamped> &plan);

  /**
    * This can be used if there are no frontiers to unknown space left in the map. The robot will retrieve it's path travelled so far via a service
    * and try to go to places having a large distance to this path.
    * @param start The start point
    * @param plan The plan to explore into unknown space
    */
  //bool doInnerExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan);

  //bool getObservationPose(const geometry_msgs::PoseStamped& observation_pose, const double desired_distance, geometry_msgs::PoseStamped& new_observation_pose);

  //bool doAlternativeExploration(const geometry_msgs::PoseStamped &start,std::vector<geometry_msgs::PoseStamped> &plan, std::vector<geometry_msgs::PoseStamped> &oldplan);
  //bool findFrontiersCloseToPath(std::vector<geometry_msgs::PoseStamped> &frontiers);
  //bool findInnerFrontier(std::vector<geometry_msgs::PoseStamped> &innerFrontier);
  


  bool setMap(const grid_map::GridMap& map);

  grid_map::GridMap& getPlanningMap()
  {
    return planning_map_;
  }

  const grid_map::GridMap& getPlanningMap() const
  {
    return planning_map_;
  }
  
  void setDistanceThresholds(double lethal_dist,
                             double penalty_dist,
                             double penalty_weight)
  {
      lethal_dist_ = lethal_dist;
      penalty_dist_ = penalty_dist;
      penalty_weight_ = penalty_weight;
  }

  void setGoalDistanceFromObstacles(double goal_dist) {
    goal_dist_from_obstacles_ = goal_dist;
  }

private:

  enum LastMode{
    FRONTIER_EXPLORE,
    INNER_EXPLORE
  } last_mode_;

  grid_map::GridMap planning_map_;

  std::vector<grid_map::Index> obstacle_cells_;
  std::vector<grid_map::Index> frontier_cells_;
  
  double lethal_dist_;
  double penalty_dist_;
  double penalty_weight_;
  double goal_dist_from_obstacles_;

};
}

#endif


