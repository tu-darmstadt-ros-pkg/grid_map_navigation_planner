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


#include <ros/ros.h>
#include <grid_map_planner_lib/grid_map_planner.h>

#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

class TestGridMapPlanner
{
public:
  TestGridMapPlanner()
  {
    ros::NodeHandle nh;

    map_sub_ = nh.subscribe("/map",10,&TestGridMapPlanner::map_cb,this);

    map_pub_ = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 2, true);
  }

  void map_cb(const nav_msgs::OccupancyGridConstPtr& grid_map_msg)
  {
    ROS_INFO("Received map");
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromOccupancyGrid(*grid_map_msg, std::string("occupancy"), map);
    gp_.setMap(map);

    grid_map_msgs::GridMap grid_map_out;
    grid_map::GridMapRosConverter::toMessage(gp_.getPlanningMap(), grid_map_out);

    std::cout << "layers: " << gp_.getPlanningMap().getLayers().size() << "\n";
    //grid_map::GridMapRosConverter::toMessage(map, grid_map_out);
    map_pub_.publish(grid_map_out);
    ROS_INFO("Map callback completed");
  }

protected:
  grid_map_planner::GridMapPlanner gp_;

  ros::Subscriber map_sub_;
  ros::Publisher map_pub_;

};

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  TestGridMapPlanner ep;

  ros::spin();

  return 0;
}
