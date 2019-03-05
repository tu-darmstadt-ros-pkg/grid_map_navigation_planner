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

#include <grid_map_planner_nav_plugin/grid_map_planner_hector_global_plugin.h>

#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(grid_map_planner_nav_plugin::GridMapPlannerGlobalPlugin, nav_core::BaseGlobalPlanner);
PLUGINLIB_EXPORT_CLASS(grid_map_planner_nav_plugin::GridMapPlannerHectorGlobalPlugin, hector_nav_core::ExplorationPlanner)

using namespace grid_map_planner_nav_plugin;


GridMapPlannerHectorGlobalPlugin::GridMapPlannerHectorGlobalPlugin()
{
  grid_map_planner_.reset(new grid_map_planner::GridMapPlanner());
}

GridMapPlannerHectorGlobalPlugin::~GridMapPlannerHectorGlobalPlugin()
{}

bool GridMapPlannerHectorGlobalPlugin::makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan,
                      const float distance)
{
std::string frame = converter_.getCostmap()->getGlobalFrameID();
  
  if(start.header.frame_id != frame) {
    ROS_ERROR_STREAM("Start passed to GridMapPlannerGlobalPlugin must be in frame " << frame << ", but is instead in frame " << start.header.frame_id);
    return false;
  }
  
  if(goal.header.frame_id != frame) {
    ROS_ERROR_STREAM("Goal passed to GridMapPlannerGlobalPlugin must be in frame " << frame << ", but is instead in frame " << goal.header.frame_id);
    return false;
  }
  
  grid_map::GridMap map;
  converter_.convertToGridMap(map);
  grid_map_planner_->setMap(map);
  bool ret = grid_map_planner_->makePlan(start.pose, goal.pose, plan);
  
  // fill up header of plan
  if(ret) {    
    for(auto& pose: plan) {
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame;
    }
  }
  
  return ret;
}

bool GridMapPlannerHectorGlobalPlugin::doExploration(const geometry_msgs::PoseStamped &start,
    std::vector<geometry_msgs::PoseStamped> &plan)
{
  grid_map::GridMap map;
  converter_.convertToGridMap(map);
  grid_map_planner_->setMap(map);
  return grid_map_planner_->makeExplorationPlan(start.pose, plan);
}

void GridMapPlannerHectorGlobalPlugin::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  //costmap_ros_ = costmap_ros;
  converter_.initialize(name, costmap_ros);
}
