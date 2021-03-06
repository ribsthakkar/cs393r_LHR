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
#include "shared/util/random.h"
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

void Experiment1(RRTVariant variant, int numExperiments=100) {
  Eigen::Vector2f startLocation = Eigen::Vector2f(-5,-5);
  Eigen::Vector2f endLocation = Eigen::Vector2f(5, 5);
  double minDistance = (startLocation - endLocation).norm() - GOAL_RADIUS;
  std::vector<Eigen::Vector2f> emptyPointCloud;
  std::vector<std::pair<double, Eigen::Vector2f>> koutput;
  vector_map::VectorMap map("maps/EmptyMap.txt");
  std::vector<Eigen::Vector2f> loutput;
  for (int scale = 1; scale <= 5; scale++)
  {
      double min_x = (startLocation.x()) * scale - 1;
      double min_y = (startLocation.y()) * scale - 1;
      double max_x = (endLocation.x()) * scale + 1;
      double max_y = (endLocation.y()) * scale + 1;
      for (int i = 0; i < numExperiments; i++) {
          auto initialTime = GetWallTime();
          auto rr_tree = rrt::RRT(startLocation, M_PI/4, endLocation, M_PI/4, std::make_pair(min_x, max_x), std::make_pair(min_y, max_y), map, visualization::NewVisualizationMessage("map", "map_lines"));
          
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
void addMapLines(const std::vector<geometry::line2f>& lines, VisualizationMsg& viz_msg, vector_map::VectorMap& map) {
  for (const auto& line : lines) {
    visualization::DrawLine(line.p0, line.p1, 0x000000, viz_msg);
    map.lines.push_back(line);
  }
}

std::pair<std::vector<geometry::line2f>, vector_map::VectorMap> setupExperiment2(int num_obstacles, const Vector2f& start, const Vector2f& end) {
  std::pair<std::vector<geometry::line2f>, vector_map::VectorMap> output;

  vector_map::VectorMap map("maps/GDC1.txt");
  output.second = map;

  const float length = 1.0;
  const std::pair<float, float> x_bounds{-42, 3};
  const std::pair<float, float> y_bounds{3, 22};

  util_random::Random rng(GetWallTime());

  output.first.reserve(num_obstacles);
  for (int i=0 ; i < num_obstacles; ++i) {
    const float heading = rng.UniformRandom(0, M_2PI);
    const Vector2f first_point(rng.UniformRandom(x_bounds.first, x_bounds.second), rng.UniformRandom(y_bounds.first, y_bounds.second));
    const Vector2f second_point = first_point + length * geometry::Heading(heading);

    // Disallow line points close to the goal
    if ((first_point-start).norm() < length || (first_point-end).norm() < length || (second_point-start).norm() < length || (second_point-end).norm() < length) {
      continue;
    }
    output.first.push_back(geometry::line2f(first_point, second_point));
  }

  return output;
}

void Experiment2(int numExperiments=50) {
  Eigen::Vector2f startLocation = Eigen::Vector2f(-40,4.5);
  Eigen::Vector2f endLocation = Eigen::Vector2f(-17, 20.5);
  const std::pair<float, float> x_bounds{-42, 3};
  const std::pair<float, float> y_bounds{3, 22};
  double minDistance = (startLocation - endLocation).norm() - GOAL_RADIUS;
  std::vector<Eigen::Vector2f> emptyPointCloud;
  std::vector<std::pair<double, Eigen::Vector2f>> koutput;
  std::vector<Eigen::Vector2f> loutput;
  for (int obs = 0; obs < 4; obs++)
  {
      const int num_obs = 50 * obs;

      for (int i = 0; i < numExperiments; i++) {

          auto lines_and_map = setupExperiment2(num_obs, startLocation, endLocation);
          VisualizationMsg global_viz_msg = visualization::NewVisualizationMessage("map", "map_lines");
          visualization::ClearVisualizationMsg(global_viz_msg);
          addMapLines(lines_and_map.first, global_viz_msg, lines_and_map.second);

          auto rr_tree_informed = rrt::RRT(startLocation, M_PI/4, endLocation, M_PI/4, x_bounds, y_bounds, lines_and_map.second, global_viz_msg);
          auto rr_tree_normal = rrt::RRT(startLocation, M_PI/4, endLocation, M_PI/4, x_bounds, y_bounds, lines_and_map.second, global_viz_msg);
          
          loutput = rr_tree_informed.LinearInformedRRT(emptyPointCloud, 100000, 0.01, minDistance, 10000);
          if (koutput.size() == 0 && loutput.size() == 0)
            printf("Could not find any path within iteration limit\n");
          printf("Obstacles(%d), Experiment (%d): (%f)\n", num_obs, i, rr_tree_informed.c_best_overall);

          loutput = rr_tree_normal.LinearRRT(emptyPointCloud, 100000, 0.01, minDistance, 10000);
          if (koutput.size() == 0 && loutput.size() == 0)
            printf("Could not find any path within iteration limit\n");
          printf("Obstacles(%d), Experiment (%d): (%f)\n", num_obs, i, rr_tree_normal.c_best_overall);

          koutput.clear();
          loutput.clear();
      }
  }

}

std::pair<std::vector<geometry::line2f>, vector_map::VectorMap> setupExperiment3(double wallHeight, double gapRatio) {
  std::pair<std::vector<geometry::line2f>, vector_map::VectorMap> output;
  double halfWallGap = (wallHeight * gapRatio)/2.0f;
  geometry::line2f upperWallTop(-1.0, wallHeight/2 + halfWallGap, 1.0, wallHeight/2 + halfWallGap);
  geometry::line2f upperWallBottom(-1.0, halfWallGap, 1.0, halfWallGap);
  geometry::line2f upperWallLeft(-1.0, wallHeight/2 + halfWallGap, -1.0, halfWallGap);
  geometry::line2f upperWallRight(1.0, wallHeight/2 + halfWallGap, 1.0, halfWallGap);

  geometry::line2f lowerWallTop(-1.0, -halfWallGap, 1.0, -halfWallGap);
  geometry::line2f lowerWallBottom(-1.0, -wallHeight/2 - halfWallGap, 1.0, -wallHeight/2 - halfWallGap);
  geometry::line2f lowerWallLeft(-1.0, -wallHeight/2 - halfWallGap, -1.0, -halfWallGap);
  geometry::line2f lowerWallRight(1.0, -wallHeight/2 - halfWallGap, 1.0, -halfWallGap);

  output.first.push_back(upperWallTop);
  output.first.push_back(upperWallBottom);
  output.first.push_back(upperWallLeft);
  output.first.push_back(upperWallRight);
  output.first.push_back(lowerWallBottom);
  output.first.push_back(lowerWallTop);
  output.first.push_back(lowerWallLeft);
  output.first.push_back(lowerWallRight);
  vector_map::VectorMap map("maps/EmptyMap.txt");
  output.second = map;

  return output;
}


void Experiment3(RRTVariant variant, int numExperiments=50)
{
  std::vector<Eigen::Vector2f> emptyPointCloud;
  std::vector<std::pair<double, Eigen::Vector2f>> koutput;
  std::vector<Eigen::Vector2f> loutput;
    double min_x = -6.0f;
    double min_y = -15.0f;
    double max_x = 6.0f;
    double max_y = 15.0f;
    Eigen::Vector2f startLocation(-5.0f, 0.0f);
    Eigen::Vector2f endLocation(5.0f, 0.0f);
    double minDistance = (startLocation - endLocation).norm() - GOAL_RADIUS;
    VisualizationMsg viz_msg = visualization::NewVisualizationMessage("map", "map_lines");

  for (int gap = 1; gap <= 5; gap++)
  {
      visualization::ClearVisualizationMsg(viz_msg);
      auto lines_and_map = setupExperiment3(20.0f, 0.01f * gap);
      addMapLines(lines_and_map.first, viz_msg, lines_and_map.second);
      for (int i = 0; i < numExperiments; i++) {
          auto initialTime = GetWallTime();
          auto rr_tree = rrt::RRT(startLocation, 0.0f, endLocation, 0.0f, std::make_pair(min_x, max_x), std::make_pair(min_y, max_y), lines_and_map.second, viz_msg);
          
          switch (variant)
          {
            case RRTVariant::KIRRT:
                koutput = rr_tree.KinodynamicInformedRRT(emptyPointCloud, 100000, 0.01, minDistance);
                break;
            case RRTVariant::LIRRT:
                loutput = rr_tree.LinearInformedRRT(emptyPointCloud, 100000, 0.01, minDistance);
                break;
            case RRTVariant::KRRT:
                koutput = rr_tree.KinodynamicRRT(emptyPointCloud, 100000, 0.01, minDistance);
                break;
            case RRTVariant::LRRT:
                loutput = rr_tree.LinearRRT(emptyPointCloud, 100000, 0.01, minDistance);
                break;
          }
          if (koutput.size() == 0 && loutput.size() == 0)
            printf("Could not find any path within iteration limit\n");
          printf("Gap (%d), Experiment (%d): (%f)\n", gap, i, GetWallTime() - initialTime);
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

  Experiment2(50);

  printf("Finished experiments\n");
  RateLoop loop(20.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    loop.Sleep();
  }
  return 0;
}
