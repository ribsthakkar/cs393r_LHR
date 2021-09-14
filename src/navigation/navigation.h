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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>
#include <deque>
#include <string>
#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "visualization/visualization.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

#define CONTROL_FREQUENCY 20.0
#define DT 1.0/CONTROL_FREQUENCY
#define WHEELBASE 0.32385
#define LENGTH 0.4
#define TRACK_WIDTH 0.1
#define WIDTH 0.15
#define SYSTEM_LATENCY 0.3
#define MAX_ACCELERATION 6.0
#define MAX_DECELERATION 6.0
#define MAX_VELOCITY 1.0
#define MIN_STEER -30.0
#define MAX_STEER 30.0
#define DSTEER 5.0

template<typename T>
std::ostream& operator<<(std::ostream& s, const std::deque<T>& v) 
{
    s.put('[');
    char comma[3] = {'\0', ' ', '\0'};
    for (const auto& e : v) {
        s << comma << e;
        comma[0] = ',';
    }
    return s << ']';
}

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

enum Collision { NONE, FRONT, INSIDE, OUTSIDE };

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Collision collision_type;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n, float system_latency, float ac_to_obs);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

 private:

  // Estimate odometry values after latency compensation
  void estimate_latency_compensated_odometry(Eigen::Vector2f* projected_loc, float* projected_angle, Eigen::Vector2f* projected_vel, float* projected_dist_traversed);
  
  // Apply latency compensated odometry transformations
  void apply_latency_compensated_odometry(Eigen::Vector2f dloc, float dangle);

  // Compute target velocity for time-optimal controller
  float compute_toc(float distance_to_target, float init_v);

  // Draws a box representing the car
  void DrawCar(uint32_t color, amrl_msgs::VisualizationMsg& msg);

  // Returns the side of the car that will collide with point, or NONE
  Collision CheckCollision(float radius, Eigen::Vector2f& point);
  Collision DebugCheckCollision(float radius, Eigen::Vector2f& point);

  // Returns the point (in base_link frame) where the point will collide with the robot
  Eigen::Vector2f GetCollisionPoint(float turn_radius, float point_radius, Collision collision_type);
  
  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Last Odometry-reported robot location.
  Eigen::Vector2f last_odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Total Distance traversed
  float odom_dist_traversed_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  // Points of interest on the car
  Eigen::Vector2f front_left_corner_;
  Eigen::Vector2f front_right_corner_;
  Eigen::Vector2f back_right_corner_;
  Eigen::Vector2f back_left_corner_;
  Eigen::Vector2f left_wheel_outside_;
  Eigen::Vector2f right_wheel_outside_;

  // Estimated system latency
  float system_latency_;
  // Actuation Latency
  float act_latency_;
  // Observation Latency
  float obs_latency_;


  //Last Issued Velocity Commands
  std::deque<float> vel_history_;
  //Last Issued Steering Commands
  std::deque<float> steer_history_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
