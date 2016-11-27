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

#include <grid_map_core/grid_map_core.hpp>


namespace grid_map_planner_goal_types{

/**
 * @brief The MapGoalBase class is the abstract base class for all map goal types
 */
class MapGoalBase
{
public:

  /**
   * @brief This functions collects all goal_indices represented by this map goal
   */
  virtual bool getGoalIndices(std::vector<grid_map::Index>& goal_indices) = 0;
  
  /**
   * @brief This functions is used to check if the provided goal index belongs to
   * this map goal.
   */
  virtual bool isReached(const grid_map::Index& reached_goal_idx) const = 0;
  
  /**
   * @brief getOrientation
   * @return The orientation used for this map goal
   */
  virtual geometry_msgs::Quaternion getOrientation() const = 0;
  
};

class PoseGoal: public MapGoalBase
{
  
public:
  PoseGoal(const geometry_msgs::Pose& pose,
           const grid_map::GridMap& grid_map)
  {
    grid_map.getIndex(grid_map::Position(pose.position.x, pose.position.y), goal_idx_);
    orientation_ = pose.orientation;
  }
  
  virtual bool getGoalIndices(std::vector<grid_map::Index>& goal_indices){
    goal_indices.push_back(goal_idx_);
    return true;
  }
  
  virtual bool isReached(const grid_map::Index& reached_goal_idx) const
  {
    return goal_idx_.matrix() == reached_goal_idx.matrix();
  }

  geometry_msgs::Quaternion getOrientation() const
  {
    return orientation_;
  }
    
private:
  grid_map::Index goal_idx_;
  geometry_msgs::Quaternion orientation_;
};

class LineSegmentGoal: public MapGoalBase
{
  
public:
  
  LineSegmentGoal(const geometry_msgs::Point& line_start_in,
                  const geometry_msgs::Point& line_end_in,
                  double yaw_in,
                  const grid_map::GridMap& grid_map)
    : grid_map_(grid_map)
  {
    goal_yaw_ = yaw_in;
    grid_map.getIndex(grid_map::Position(line_start_in.x, line_start_in.y), line_start);
    grid_map.getIndex(grid_map::Position(line_end_in.x, line_end_in.y), line_end);
  }
  
  virtual bool getGoalIndices(std::vector<grid_map::Index>& goal_indices)
  {
    for (grid_map::LineIterator iterator (grid_map_, line_start, line_end);
         !iterator.isPastEnd(); ++iterator) {

      const grid_map::Index index(*iterator);

      //std::cout << "goal idx:\n" << index << "\n";

      goal_indices.push_back(index);
    }
    
    return true;
  }
  
  virtual bool isReached(const grid_map::Index& reached_goal_idx) const
  {
    //std::cout << "reached_goal idx:\n" << reached_goal_idx << "\n";
    for (grid_map::LineIterator iterator (grid_map_, line_start, line_end);
         !iterator.isPastEnd(); ++iterator) {

       const grid_map::Index index(*iterator);

       //std::cout << "test idx:\n" << index << "\n";


       if (index.matrix() == reached_goal_idx.matrix()){
         return true;
       }
    }
    return false;
  }

  geometry_msgs::Quaternion getOrientation() const
  {
    geometry_msgs::Quaternion quat;

    quat.z = sin(goal_yaw_*0.5f);
    quat.w = cos(goal_yaw_*0.5f);

    return quat;
  }

private:
  grid_map::Index line_start;
  grid_map::Index line_end;
  double goal_yaw_;
  const grid_map::GridMap& grid_map_;
};



}
#endif


