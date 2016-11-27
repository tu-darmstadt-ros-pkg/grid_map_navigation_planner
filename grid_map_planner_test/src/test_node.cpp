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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <nav_msgs/Path.h>

class TestGridMapPlanner
{
public:
  TestGridMapPlanner()
  {
    ros::NodeHandle nh;

    map_sub_ = nh.subscribe("/map",10,&TestGridMapPlanner::map_cb,this);

    map_pub_ = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 2, true);
    path_pub_ = nh.advertise<nav_msgs::Path>("/path", 2, true);


    goal_pose_sub_ = nh.subscribe("/goal", 2, &TestGridMapPlanner::goalPoseCallback, this);
    search_pose_sub_ = nh.subscribe("/search_pose", 2, &TestGridMapPlanner::searchPoseCallback, this);
    pose_with_cov_sub_ = nh.subscribe("/initialpose", 2, &TestGridMapPlanner::poseWithCovCallback, this);
    exploration_start_pose_sub_ = nh.subscribe("/explore_initialpose", 2, &TestGridMapPlanner::explorePoseCallback, this);
    multi_goal_start_pose_sub_ = nh.subscribe("/multi_goal_initialpose", 2, &TestGridMapPlanner::multiGoalPoseCallback, this);

  }

  void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    ROS_INFO("Goal pose callback");

    goal_pose = msg->pose;
    this->plan_path_to_goal();
  }

  void searchPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    ROS_INFO("Search pose callback");
  }

  void poseWithCovCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    ROS_INFO("Start pose callback");
    start_pose = msg->pose.pose;
    this->plan_path_to_goal();
  }

  void explorePoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    ROS_INFO("Explore pose callback");

    ros::WallTime start_time = ros::WallTime::now();

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    gp_.makeExplorationPlan(msg->pose.pose, path.poses);

    path_pub_.publish(path);

    grid_map_msgs::GridMap grid_map_out;
    grid_map::GridMapRosConverter::toMessage(gp_.getPlanningMap(), grid_map_out);
    std::cout << "Planning exploration took " << (ros::WallTime::now() - start_time).toSec() * 1000 << " ms\n";

    //grid_map::GridMapRosConverter::toMessage(map, grid_map_out);
    map_pub_.publish(grid_map_out);
  }

  void multiGoalPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    ROS_INFO("Multi goal pose callback");

    ros::WallTime start_time = ros::WallTime::now();

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    std::vector<boost::shared_ptr<grid_map_planner_goal_types::MapGoalBase> > map_goals;

    geometry_msgs::Point line_start;
    line_start.x = 20;
    line_start.y = -0.5;

    geometry_msgs::Point line_end;
    line_end.x = 20;
    line_end.y = -2;


    map_goals.push_back(boost::make_shared<grid_map_planner_goal_types::LineSegmentGoal>(
                                            line_start,
                                            line_end,
                                            0.0,
                                            gp_.getPlanningMap()));
                                            
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = 12.51;
    goal_pose.position.y = -6.1;
    goal_pose.orientation.w = 1.0;
    map_goals.push_back(boost::make_shared<grid_map_planner_goal_types::PoseGoal>(
                                            goal_pose,
                                            gp_.getPlanningMap()));


    int reached_idx;
    gp_.makePlan(msg->pose.pose, map_goals, path.poses, reached_idx);

    path_pub_.publish(path);

    grid_map_msgs::GridMap grid_map_out;
    grid_map::GridMapRosConverter::toMessage(gp_.getPlanningMap(), grid_map_out);
    std::cout << "Planning multi goal path took " << (ros::WallTime::now() - start_time).toSec() * 1000 << " ms\n";

    //grid_map::GridMapRosConverter::toMessage(map, grid_map_out);
    map_pub_.publish(grid_map_out);
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

  void plan_path_to_goal()
  {
    ROS_INFO("Plan path to goal");

    ros::WallTime start_time = ros::WallTime::now();
    //std::vector<geometry_msgs::PoseStamped> path;
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    if (!gp_.makePlan(start_pose, goal_pose ,path.poses)){
      ROS_WARN("makePlan failed");
    }

    std::cout << "Generating plan took " << (ros::WallTime::now() - start_time).toSec() * 1000 << " ms\n";

    path_pub_.publish(path);

    start_time = ros::WallTime::now();
    grid_map_msgs::GridMap grid_map_out;
    grid_map::GridMapRosConverter::toMessage(gp_.getPlanningMap(), grid_map_out);

    //grid_map::GridMapRosConverter::toMessage(map, grid_map_out);
    map_pub_.publish(grid_map_out);
    std::cout << "Publishing map took " << (ros::WallTime::now() - start_time).toSec() * 1000 << " ms\n";
  }

protected:
  grid_map_planner::GridMapPlanner gp_;

  ros::Subscriber map_sub_;

  ros::Publisher map_pub_;
  ros::Publisher path_pub_;

  ros::Subscriber goal_pose_sub_;
  ros::Subscriber search_pose_sub_;
  ros::Subscriber pose_with_cov_sub_;
  ros::Subscriber exploration_start_pose_sub_;
  ros::Subscriber multi_goal_start_pose_sub_;

  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose goal_pose;

};

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  /*
  // Quickly testing storage order :)
  Eigen::Matrix<float, 3, 3> matrix ;
  matrix << 0, 1, 2, 3, 4 ,5 ,6 ,7 , 8;
  std::cout << "\n" << matrix << "\n";
  std::cout << "0,0: " << matrix(0,0) << " 0,1: " << matrix(0,1) << " 1,0: " << matrix(1,0) << "\n";
  std::cout << "In memory:" << std::endl;
  for (int i = 0; i < matrix.size(); i++)
    std::cout << *(matrix.data() + i) << "  ";
  std::cout << std::endl << std::endl;
  */


  TestGridMapPlanner ep;

  ros::spin();

  return 0;
}
