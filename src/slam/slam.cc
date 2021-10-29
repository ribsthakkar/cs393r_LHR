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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

CONFIG_INT(dloc_count, "odometry_estimation.linear_count");
CONFIG_FLOAT(dloc_delta, "odometry_estimation.linear_precision");
CONFIG_INT(dangle_count, "odometry_estimation.angular_count");
CONFIG_FLOAT(dangle_delta, "odometry_estimation.angular_precision");
CONFIG_FLOAT(k1, "motion_model.k1");
CONFIG_FLOAT(k2, "motion_model.k2");
CONFIG_FLOAT(k3, "motion_model.k3");
CONFIG_FLOAT(k4, "motion_model.k4");
CONFIG_INT(rasterization_precision, "rasterization.radius");
CONFIG_INT(rasterization_square_size, "rasterization.square_size");
CONFIG_FLOAT(lidar_variance, "lidar_variance");

config_reader::ConfigReader config_reader_({"config/slam.lua"});

namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    poses_a_loc(0, 0),
    poses_a_angle(0),
    odom_initialized_(false),
    poses_locs(),
    poses_angles(),
    scans(),
    distance_traveled(0),
    angle_traveled(0),
    constructed_map(),
    observation_probabilities() {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = poses_locs.back();
  *angle = poses_angles.back();
}

void SLAM::Index2Delta(int ix, int iy, int itheta, int* dx, int* dy, int* dtheta)
{
  // Convert the discrete index to a float representation of the deviation values around our odometry
  *dx = CONFIG_dloc_delta * (ix - CONFIG_dloc_count/2);
  *dx = CONFIG_dloc_delta * (iy - CONFIG_dloc_count/2);
  *dtheta = CONFIG_dangle_delta * (itheta - CONFIG_dangle_count/2);
}

float SLAM::ComputeObservationWeight(Eigen::Vector2f loc, float angle, std::vector<Eigen::Vector2f>& scan)
{
  float nll = 0.0f
  for(Vector2f p: scan)
  {
    Vector2f transformed = Eigen::Rotation2Df(angle)*p + loc;
    if (observation_probabilities.find() != observation_probabilities.end()) {
      nll += observation_probabilities[std::make_pair(round(transformed.x()*CONFIG_rasterization_precision), round(transformed.y()*CONFIG_rasterization_precision))];
    } else {
      // We need to add a big positive number since we basically are saying that htis point doesnt exist in our observation likelihood map
      // and we are minimizing the log likelihood
      nll += 100;
    }
  }
  return nll;
}

float SLAM::ComputeMotionWeight(float dx, float dy, float dtheta)
{
  // Use motion model constants to compute the error in our odometry being the input arguments
  float nll = 0.0f;
  Vector2f delta_pos(dx, dy);
  float linear_variance = CONFIG_k1*delta_pos.norm() + CONFIG_k2 * fabs(dtheta);
  float angle_variance = CONFIG_k3*delta_pos.norm() + CONFIG_k4 * fabs(dtheta);
  nll += -log(1/(linear_variance * sqrt(2*M_PI))) + ((dx*dx)/(2*linear_variance));
  nll += -log(1/(linear_variance * sqrt(2*M_PI))) + ((dy*dy)/(2*linear_variance));
  nll += -log(1/(angle_variance * sqrt(2*M_PI))) + ((dtheta*dtheta)/(2*angle_variance));
  return nll;
}

void SLAM::UpdateObservationLikelihoods()
{
  for (Vector2f p: scans.back())
  {
    // Project our float position to some precision (10, 100, 1000, etc.)
    int coarse_x = p.x() * CONFIG_rasterization_precision;
    int coarse_y = p.y() * CONFIG_rasterization_precision;
    // Loop through nearby points and give them 
    for (int x = coarse_x - CONFIG_rasterization_square_size; x <= coarse_x + CONFIG_rasterization_square_size; ++x) {
      for (int y = coarse_x - CONFIG_rasterization_square_size; y <= coarse_x + CONFIG_rasterization_square_size; ++y) {

        observation_probabilities[make_pair(x, y)] += powf(Eigen::Vector2f((x-coarse_x)/CONFIG_rasterization_precision, (y-coarse_y)/CONFIG_rasterization_precision).norm(), 2)/(2*CONFIG_lidar_variance);
      }
    }
  }
}

void SLAM::ObserveLaser(const std::vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  if(distance_traveled > 0.5 || angle_traveled > math_util::DegToRad(30) || poses.size() == 0)
  {
    Eigen::Vector2f dloc = prev_odom_loc_ - poses_a_loc;
    float dangle = prev_odom_angle_ - poses_a_angle;

    scans.push_back(std::vector<Eigen::Vector2f>());
    // Convert the LaserScan to a point cloud
    for (uint i = 0; i < msg.ranges.size(); ++i) {
      float theta = msg.angle_min + i*msg.angle_increment;
      // Transform to base link of robot by adding the laser location position
      if (msg.ranges[i]>msg.range_max || msg.ranges[i]<msg.range_min) continue;
      scans.back().push_back(Vector2f(cos(theta)*msg.ranges[i], sin(theta)*msg.ranges[i]) + kLaserLoc);
    }
    if (poses.size() > 1)
    {
      // Construct cube around dangle and dloc and select best pose
      float best_weight = std::numeric_limits<float>::infinity();
      Eigen::Vector2f best_loc = poses_locs.back();
      float best_angle = poses_angles.back();

      for(int ix = 0; ix < CONFIG_dloc_count; ix += 1) {
        for (int iy = 0; iy < CONFIG_dloc_count; iy += 1) {
          for (int itheta = 0; itheta < CONFIG_dangle_count; itheta += 1) {
              // compute the dx, dy, dtheta for this element in our Cube
              float dx, dy, dtheta;
              Index2Delta(ix, iy, itheta, *dx, *dy, *dtheta);
              
              // Determine the true location and angle being considered in the map frame and determine nll weight
              Vector2f considered_loc = poses_locs.back() + dloc + Vector2f(dx, dy);
              float considered_angle = poses_angles.back() + dangle + dtheta;
              float pose_weight = ComputeObservationWeight(considered_loc, considered_angle, scans.back()) + ComputeMotionWeight(dx, dy, dtheta);
              if (pose_weight < best_weight)
              {
                best_loc = considered_loc;
                best_angle = best_angle;
              }
          }
        }
      }
      poses_locs.push_back(best_loc);
      poses_angles.push_back(best_angle);
      // Use best pose to align last scan to the last-1th scan
      for (int i = 0; i < scans.back().size(); ++i)
      {
        scans.back()[i] = scans.back()[i]* Eigen::Rotation2Df(poses_angles.back()) + poses_locs.back();
      }
    }
    else
    {
      poses_locs.push_back(Vector2f(0,0));
      poses_angles.push_back(0.0f);
    }
    // Update Map
    for(Eigen::Vector2f p: scans.back())
    {
      map.push_back(p);
    }
    // Update Observation Likelihood Lookup-Table
    UpdateObservationLikelihoods();

    // Update prev odom pose to compute delta next time
    poses_a_angle = prev_odom_angle_;
    poses_a_loc = prev_odom_loc_;
  }
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
  Eigen::Vector2f delta_pos = (odom_loc - prev_odom_loc_);
  float delta_angle = math_util::AngleMod(odom_angle - prev_odom_angle_);
  distance_traveled += delta_pos.norm();
  angle_traveled += abs(delta_angle);
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

std::vector<Eigen::Vector2f> SLAM::GetMap() {
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
