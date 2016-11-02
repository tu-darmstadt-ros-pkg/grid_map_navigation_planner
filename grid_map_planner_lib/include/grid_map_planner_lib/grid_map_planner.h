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

//#include <dynamic_reconfigure/server.h>


//#include <boost/shared_array.hpp>

namespace grid_map_planner{

//class ExplorationPlannerConfig;

class GridMapPlanner {
public:
  GridMapPlanner();
  ~GridMapPlanner();
  //HectorExplorationPlanner(std::string name,costmap_2d::Costmap2DROS *costmap_ros);
  //void initialize(std::string name,costmap_2d::Costmap2DROS *costmap_ros);

  //void dynRecParamCallback(hector_exploration_planner::ExplorationPlannerConfig &config, uint32_t level);

  /**
   * Plans from start to given goal. If orientation quaternion of goal is all zeros, calls exploration instead. This is a hacky workaround that
   * has to be refactored.
   * @param start The start point
   * @param goal The goal point (Use orientation quaternion all 0 to let exploration find goal point)
   * @param plan The generated plan
   */
  //bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &original_goal, std::vector<geometry_msgs::PoseStamped> &plan);

  /**
    * Given a start point, finds a frontier between known and unknown space and generates a plan to go there
    * @param start The start point
    * @param plan The plan to explore into unknown space
    */
  //bool doExploration(const geometry_msgs::PoseStamped &start,std::vector<geometry_msgs::PoseStamped> &plan);

  /**
    * This can be used if there are no frontiers to unknown space left in the map. The robot will retrieve it's path travelled so far via a service
    * and try to go to places having a large distance to this path.
    * @param start The start point
    * @param plan The plan to explore into unknown space
    */
  //bool doInnerExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan);

  //bool getObservationPose(const geometry_msgs::PoseStamped& observation_pose, const double desired_distance, geometry_msgs::PoseStamped& new_observation_pose);

  //bool doAlternativeExploration(const geometry_msgs::PoseStamped &start,std::vector<geometry_msgs::PoseStamped> &plan, std::vector<geometry_msgs::PoseStamped> &oldplan);
  //bool findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers, std::vector<geometry_msgs::PoseStamped> &noFrontiers);
  //bool findFrontiersCloseToPath(std::vector<geometry_msgs::PoseStamped> &frontiers);
  //bool findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers);
  //bool findInnerFrontier(std::vector<geometry_msgs::PoseStamped> &innerFrontier);
  


  void setMap(const grid_map::GridMap& map);

private:

  enum LastMode{
    FRONTIER_EXPLORE,
    INNER_EXPLORE
  } last_mode_;

  grid_map::GridMap planning_map_;



};
}

#endif


