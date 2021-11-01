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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>
#include <map>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max,
                    float angle_increment);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

 private:

  void Index2Delta(int ix, int iy, int itheta, float* dx, float* dy, float* dtheta);
  float ComputeObservationWeight(Eigen::Vector2f loc, float angle, std::vector<Eigen::Vector2f>& scan);
  float ComputeMotionWeight(float dx, float dy, float dtheta);
  void UpdateObservationLikelihoods();
  void UpdateMap();

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  Eigen::Vector2f poses_a_loc;
  float poses_a_angle;
  bool odom_initialized_;

  // All scans
  std::vector<Eigen::Vector2f> poses_locs;
  std::vector<float> poses_angles;
  std::vector<std::vector<Eigen::Vector2f>> scans;

  //for keeping track of successive poses
  float distance_traveled;
  float angle_traveled;

  // Constructed Map
  std::vector<Eigen::Vector2f> constructed_map;
  std::map<std::pair<int, int>, float> observation_probabilities;
};
}  // namespace slam

#endif   // SRC_SLAM_H_
