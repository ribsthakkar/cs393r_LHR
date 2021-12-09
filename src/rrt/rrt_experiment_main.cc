//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation_main.cc
\brief   Main entry point for reference Navigation implementation
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"

#include "navigation.h"

using amrl_msgs::Localization2DMsg;
using math_util::DegToRad;
using math_util::RadToDeg;
using navigation::Navigation;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;

// Create command line arguments
bool run_ = true;

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

enum class RRTVariant {KIRRT, LIRRT, KRRT, LRRT};

void Experiment1(RRTVaraint variant, int numExperiments=100) {
  Eigen::Vector2f startLocation = Eigen::Vector2f(-5,-5);
  Eigen::Vector2f endLocation = Eigen::Vector2f(5, 5);
  double minDistance = (startLocation - endLocation).norm();
  vector_map::VectorMap map("maps/EmptyMap.txt");
  std::vector<Eigen::Vector2f> emptyPointCloud;
  std::vector<std::pair<double, Eigen::Vector2f>> koutput;
  std::vector<Eigen::Vector2f> loutput;
  for (int scale = 1; scale < 4; scale++)
  {
      double min_x = startLocation.x() * scale;
      double min_y = startLocation.y() * scale;
      double max_x = endLocation.x() * scale;
      double max_y = endLocation.y() * scale;
      for (int i = 0; i < numExperiments; i++) {
          auto initialTime = GetWallTime();
          auto rr_tree = rrt::RRT(startLocation, M_PI/2, endLocation, M_PI/2, std::make_pair(min_x, max_x), std::make_pair(min_y, max_y), map);
          
          switch (variant)
          {
            case KIRRT:
                koutput = rr_tree.KinodynamicInformedRRT(emptyPointCloud, max_iterations=1000000, costGap=0.01, optimalCost=minDistance);
                break;
            case LIRRT:
                loutput = rr_tree.LinearInformedRRT(emptyPointCloud, max_iterations=1000000, costGap=0.01, optimalCost=minDistance);
                break;
            case KRRT:
                koutput = rr_tree.KinodynamicRRT(emptyPointCloud, max_iterations=1000000, costGap=0.01, optimalCost=minDistance);
                break;
            case LRRT:
                loutput = rr_tree.LinearRRT(emptyPointCloud, max_iterations=1000000, costGap=0.01, optimalCost=minDistance);
                break;
          }
          if (koutput.size() == 0 && loutput.size() == 0)
            printf("Could not find any path within iteration limit");
          printf("Scale(%d), Experiment (%d): (%f)", scale, i, GetWallTime() - initialTime);
          koutput.clear();
          loutput.clear();
      }
  }

}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "rrt_experiment", ros::init_options::NoSigintHandler);
  
  Experiment1(RRTVaraint.LIRRT);
  Experiment1(RRTVaraint.LRRT);


  RateLoop loop(20.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    loop.Sleep();
  }
  return 0;
}
