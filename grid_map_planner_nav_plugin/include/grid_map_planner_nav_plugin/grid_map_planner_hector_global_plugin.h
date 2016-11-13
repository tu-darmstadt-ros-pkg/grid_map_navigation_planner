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

#ifndef GRID_MAP_PLANNER_GLOBAL_PLUGIN_H___
#define GRID_MAP_PLANNER_GLOBAL_PLUGIN_H___

#include <grid_map_planner_nav_plugin/grid_map_cost_map_2d_conversion.h>

#include <grid_map_planner_lib/grid_map_planner.h>

#include <hector_nav_core/exploration_planner.h>
#include <pluginlib/class_list_macros.h>


namespace grid_map_planner_nav_plugin{

class GridMapPlannerHectorGlobalPlugin : public hector_nav_core::ExplorationPlanner
{
public:
  GridMapPlannerHectorGlobalPlugin();
  virtual ~GridMapPlannerHectorGlobalPlugin();


  virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan,
                        const float distance);

  virtual bool doExploration(const geometry_msgs::PoseStamped &start,
      std::vector<geometry_msgs::PoseStamped> &plan);

  virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

protected:  
  boost::shared_ptr<grid_map_planner::GridMapPlanner> grid_map_planner_;
  //costmap_2d::Costmap2DROS* costmap_ros_;

  grid_map_costmap_2d_conversion::GridMapCostmap2DConversion converter_;
};


}

#endif
