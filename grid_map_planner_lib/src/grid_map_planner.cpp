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
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace grid_map_planner;

  GridMapPlanner::GridMapPlanner()
  {

  }
  
  
  GridMapPlanner::~GridMapPlanner()
  {
  }

  void GridMapPlanner::setMap(const grid_map::GridMap& map)
  {
   ros::WallTime start_time = ros::WallTime::now();

   // timing: 1.0ms
   this->planning_map_ = map;

   //this->planning_map_.add("distance_transform");

   // See https://github.com/opencv/opencv/blob/05b15943d6a42c99e5f921b7dbaa8323f3c042c6/modules/imgproc/src/distransform.cpp

   grid_map::Matrix& grid_data = this->planning_map_["occupancy"];
   cv::Mat map_mat = cv::Mat::zeros(planning_map_.getSize()(0), planning_map_.getSize()(1), CV_8UC1);;


   //cv::eigen2cv(planning_map_.get("occupancy"), map_mat);
   // timing: ~10ms
   //grid_map::GridMapCvConverter::toImage<unsigned char, 1>(this->planning_map_,std::string("occupancy"),CV_8UC1, 1.0f, 0.0f, map_mat);


   float lowerValue = 1.0;
   float upperValue = 0.0;
   start_time = ros::WallTime::now();

   uchar *input = (uchar*)(map_mat.data);

   float inv_up_subtr_low = 1.0 / (upperValue - lowerValue);

   grid_map::Index index;
   for (index(0) = 0; index(0) < planning_map_.getSize()(0); ++index(0)){
     for (index(1) = 0; index(1) < planning_map_.getSize()(1); ++index(1)){

           //for (grid_map::GridMapIterator iterator(planning_map_); !iterator.isPastEnd(); ++iterator) {
           //const grid_map::Index index(*iterator);

           //data(index(0), index(1)) = distance_transformed.at<float>(index(0), index(1));
           //data(index(0), index(1)) =
           //const float& value = ;
           //const uchar imageValue = (uchar) (((grid_data(index(0), index(1)) - lowerValue) / (upperValue - lowerValue)) * (float) 255);

       input[map_mat.cols * index(0) + index(1)] = (uchar) (((grid_data(index(0), index(1)) - lowerValue) * inv_up_subtr_low) * (float) 255);
     }
   }
   //}
   std::cout << "Took " << (ros::WallTime::now() - start_time).toSec() * 1000 << " ms\n";





   //map_mat.at<uchar>(100, 100) = 0;

   cv::namedWindow("orig");
   //map_mat.at<uchar>(101, 101) = 255;
   cv::imshow("orig", map_mat);
   //cv::waitKey();

   std::string dist_trans_layer = "distance_transform";

   this->planning_map_.add(dist_trans_layer);
   grid_map::Matrix& data = this->planning_map_[dist_trans_layer];

   cv::Mat distance_transformed (data.rows(), data.cols(), CV_32FC1, data.data());

   //Mat B_OpenCV(B.rows(), B.cols(), CV_32FC1, B.data());
   //cv::eigen2cv(planning_map_.get(dist_trans_layer), distance_transformed);


   //map_mat[50000] = 0;

   // @TODO Appears OpenCV 2.4 broken in that it does not provide enums. Looked up and manually added values
   // https://github.com/opencv/opencv/blob/master/modules/imgproc/include/opencv2/imgproc.hpp#L308
   cv::distanceTransform(map_mat, distance_transformed, 2, 3);





   double min;
   double max;
   cv::minMaxIdx(distance_transformed, &min, &max);
   std::cout << "min: "<< min <<  " max: " << max << "\n";
   cv::Mat adjMap;
   // Histogram Equalization
   float scale = 255 / (max-min);
   distance_transformed.convertTo(adjMap,CV_8UC1, scale, -min*scale);

   cv::namedWindow("bla");
   cv::imshow("bla", adjMap);
   cv::waitKey();

   //data = data.transpose();

   //grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(distance_transformed, std::string("distance_transform"), this->planning_map_, 0.0, max);

   /*
   std::string dist_trans_layer = "distance_transform";

   this->planning_map_.add(dist_trans_layer);
   grid_map::Matrix& data = this->planning_map_[dist_trans_layer];
   start_time = ros::WallTime::now();

   float *input = (float*)(distance_transformed.data);


   for (grid_map::GridMapIterator iterator(planning_map_); !iterator.isPastEnd(); ++iterator) {
     const grid_map::Index index(*iterator);

     //data(index(0), index(1)) = distance_transformed.at<float>(index(0), index(1));
     data(index(0), index(1)) = input[distance_transformed.cols * index(1) + index(0)];
   }
   */

   std::cout << "Took " << (ros::WallTime::now() - start_time).toSec() * 1000 << " ms\n";


  }
