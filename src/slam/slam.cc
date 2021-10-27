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

namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    poses_locs(),
    poses_angles(),
    scans(),
    map() {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = poses_locs.back();
  *angle = poses_angles.back();
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  if(distance_traveled > 0.5 || angle_traveled > math_util::DegToRad(30) || poses.size() == 0)
  {
    scans.push_back(vector<Vector2f>());
    // Convert the LaserScan to a point cloud
    for (uint i = 0; i < msg.ranges.size(); ++i) {
      float theta = msg.angle_min + i*msg.angle_increment;
      // Transform to base link of robot by adding the laser location position
      if (msg.ranges[i]>msg.range_max || msg.ranges[i]<msg.range_min) continue;
      scans.back().push_back(Vector2f(cos(theta)*msg.ranges[i], sin(theta)*msg.ranges[i]) + kLaserLoc);
    }
    if (poses.size() > 1)
    {
      // Align last scan to the last-1th scan

      // Determine the new pose

      // Update map
    } 
    else
    {
      for(Vector2f p: scans.back())
      {
        map.push_back(p);
      }
      poses_locs.push_back(Vector2f(0,0));
      poses_angles.push_back(0.0f);
    }
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
  Vector2f delta_pos = Eigen::Rotation2Df(-1*prev_odom_angle_) * (odom_loc - prev_odom_loc_);
  float delta_angle = math_util::AngleMod(odom_angle - prev_odom_angle_);
  
  distance_traveled += delta_pos.norm();
  angle_traveled += abs(delta_angle);
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

vector<Vector2f> SLAM::GetMap() {
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
