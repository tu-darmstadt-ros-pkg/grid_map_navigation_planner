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

#ifndef GRID_MAP_PLANNER_GOAL_TYPES_H___
#define GRID_MAP_PLANNER_GOAL_TYPES_H___


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <grid_map_core/GridMap.hpp>


namespace grid_map_planner_goal_types{


class MapGoalBase
{
public:
  virtual bool addToExplorationGoalLayer(grid_map::Matrix& exploration_grid_data) = 0;
  
  virtual bool checkIfReached(const grid_map::Matrix& exploration_grid_data) = 0;
  
  virtual bool isSingleGoal() = 0;
  
};

class PoseGoal: public MapGoalBase
{
  
public:
  PoseGoal(const geometry_msgs::Pose pose)
  : goal (pose)
  {}
  
  virtual bool addToExplorationGoalLayer(grid_map::Matrix& exploration_grid_data){
    
  }
  
  virtual bool checkIfReached(const grid_map::Matrix& exploration_grid_data){
    
  }
  
  virtual bool isSingleGoal(){
    return false;
  }
  
private:
  geometry_msgs::Pose goal;
};

class LineSegmentGoal: public MapGoalBase
{
  
public:
  
  LineSegmentGoal(const geometry_msgs::Point& line_start_in,
                  const geometry_msgs::Point& line_end_in,
                  double yaw_in)
  : line_start(line_start_in)
  , line_end(line_end_in)
  , yaw(yaw_in)
  {}
  
  virtual bool addToExplorationGoalLayer(grid_map::Matrix& exploration_grid_data)
  {
    
  }
  
  virtual bool checkIfReached(const grid_map::Matrix& exploration_grid_data)
  {
    
  }
  
  virtual bool isSingleGoal(){
    return false;
  }
  
private:
  geometry_msgs::Point line_start;
  geometry_msgs::Point line_end;
  float yaw;  
};



}
#endif


