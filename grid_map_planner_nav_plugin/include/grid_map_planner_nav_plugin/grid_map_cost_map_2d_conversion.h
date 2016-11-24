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

#ifndef GRID_MAP_COSTMAP2D_CONVERSION_H___
#define GRID_MAP_COSTMAP2D_CONVERSION_H__

#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <costmap_2d/costmap_2d_ros.h>


namespace grid_map_costmap_2d_conversion{

class GridMapCostmap2DConversion
{
public:
  GridMapCostmap2DConversion()
  {}

  ~GridMapCostmap2DConversion()
  {}
  
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    {
      cost_translation_table_.resize(256);

      // special values:
      cost_translation_table_[0] = 0;  // NO obstacle
      cost_translation_table_[253] = 99;  // INSCRIBED obstacle
      cost_translation_table_[254] = 100;  // LETHAL obstacle
      cost_translation_table_[255] = -1;  // UNKNOWN

      // regular cost values scale the range 1 to 252 (inclusive) to fit
      // into 1 to 98 (inclusive).
      for (int i = 1; i < 253; i++)
      {
        cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
      }
    }

    this->costmap_ros_ = costmap_ros;
  }

  bool convertToGridMap(grid_map::GridMap& grid_map_out)
  {
    costmap_2d::Costmap2D* costmap_ = this->costmap_ros_->getCostmap();

    double resolution = costmap_->getResolution();

    grid_.header.frame_id = this->costmap_ros_->getGlobalFrameID();
    grid_.header.stamp = ros::Time::now();
    grid_.info.resolution = resolution;

    grid_.info.width = costmap_->getSizeInCellsX();
    grid_.info.height = costmap_->getSizeInCellsY();

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid_.info.origin.position.x = wx - resolution / 2;
    grid_.info.origin.position.y = wy - resolution / 2;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;
    //saved_origin_x_ = costmap_->getOriginX();
    //saved_origin_y_ = costmap_->getOriginY();

    grid_.data.resize(grid_.info.width * grid_.info.height);

    unsigned char* data = costmap_->getCharMap();
    for (unsigned int i = 0; i < grid_.data.size(); i++)
    {
      grid_.data[i] = cost_translation_table_[ data[ i ]];
    }

    return grid_map::GridMapRosConverter::fromOccupancyGrid(grid_, "occupancy", grid_map_out);
  }
  
  costmap_2d::Costmap2DROS* getCostmap() {
    return costmap_ros_;
  }
  
protected:  
  costmap_2d::Costmap2DROS* costmap_ros_;
  nav_msgs::OccupancyGrid grid_;

  std::vector<char> cost_translation_table_;
};


}

#endif
