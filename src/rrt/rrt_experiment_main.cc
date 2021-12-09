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
#include "amrl_msgs/VisualizationMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "vector_map/vector_map.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "rrt/rrt.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "visualization/visualization.h"

using amrl_msgs::Localization2DMsg;
using math_util::DegToRad;
using math_util::RadToDeg;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;
using amrl_msgs::VisualizationMsg;

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

enum RRTVariant {KIRRT, LIRRT, KRRT, LRRT};

void Experiment1(RRTVariant variant, const vector_map::VectorMap& map, int numExperiments=100) {
  Eigen::Vector2f startLocation = Eigen::Vector2f(-5,-5);
  Eigen::Vector2f endLocation = Eigen::Vector2f(5, 5);
  double minDistance = (startLocation - endLocation).norm() - GOAL_RADIUS;
  vector_map::VectorMap map_("maps/EmptyMap.txt");
  std::vector<Eigen::Vector2f> emptyPointCloud;
  std::vector<std::pair<double, Eigen::Vector2f>> koutput;
  std::vector<Eigen::Vector2f> loutput;
  for (int scale = 1; scale < 4; scale++)
  {
      double min_x = (startLocation.x()) * scale - 1;
      double min_y = (startLocation.y()) * scale - 1;
      double max_x = (endLocation.x()) * scale + 1;
      double max_y = (endLocation.y()) * scale + 1;
      for (int i = 0; i < numExperiments; i++) {
          auto initialTime = GetWallTime();
          auto rr_tree = rrt::RRT(startLocation, M_PI/4, endLocation, M_PI/4, std::make_pair(min_x, max_x), std::make_pair(min_y, max_y), map);
          
          switch (variant)
          {
            case RRTVariant::KIRRT:
                koutput = rr_tree.KinodynamicInformedRRT(emptyPointCloud, 1000000, 0.01, minDistance);
                break;
            case RRTVariant::LIRRT:
                loutput = rr_tree.LinearInformedRRT(emptyPointCloud, 1000000, 0.01, minDistance);
                break;
            case RRTVariant::KRRT:
                koutput = rr_tree.KinodynamicRRT(emptyPointCloud, 1000000, 0.01, minDistance);
                break;
            case RRTVariant::LRRT:
                loutput = rr_tree.LinearRRT(emptyPointCloud, 1000000, 0.01, minDistance);
                break;
          }
          if (koutput.size() == 0 && loutput.size() == 0)
            printf("Could not find any path within iteration limit\n");
          printf("Scale(%d), Experiment (%d): (%f)\n", scale, i, GetWallTime() - initialTime);
          koutput.clear();
          loutput.clear();
      }
  }

}

// Adds lines to the visualization message and Vectormap
void addMapLines(const std::vector<geometry::line2d>& lines, VisualizationMsg& viz_msg, vector_map::VectorMap& map) {
  // TODO: pass vector of lines and then 
  visualization::DrawLine(Vector2f(0,0), Vector2f(10,10), 0x000000, viz_msg);
  (void) map;
  (void) lines;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "rrt_experiment", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::Publisher viz_pub = nh.advertise<VisualizationMsg>("visualization", 1);
  VisualizationMsg global_viz_msg = visualization::NewVisualizationMessage("map", "map_lines");

  std::vector<geometry::line2d> obstacles;
  vector_map::VectorMap map("maps/EmptyMap.txt");
  addMapLines(obstacles, global_viz_msg, map);

  global_viz_msg.header.stamp = ros::Time::now();
  viz_pub.publish(global_viz_msg);

  // Experiment1(RRTVariant::LIRRT, map);
  // Experiment1(RRTVariant::LRRT, map);
  // Experiment1(RRTVariant::KIRRT, map, 1);
  // Experiment1(RRTVariant::KRRT, map, 1);

  RateLoop loop(20.0);
  while (run_ && ros::ok()) {
    global_viz_msg.header.stamp = ros::Time::now();
    viz_pub.publish(global_viz_msg);
    ros::spinOnce();
    loop.Sleep();
  }
  return 0;
}
