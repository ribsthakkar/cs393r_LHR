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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using Eigen::Rotation2Df;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n, float system_latency, float ac_to_obs) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    last_odom_loc_(0, 0),
    odom_dist_traversed_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    system_latency_(system_latency),
    act_latency_((ac_to_obs/(ac_to_obs+1.0))*system_latency),
    obs_latency_((1.0/(ac_to_obs+1.0))*system_latency) {
  
  uint history_length = static_cast<uint>(CONTROL_FREQUENCY * system_latency) + 1;
  vel_history_ = std::deque<float>(history_length, 0.0);
  steer_history_ = std::deque<float>(history_length, 0.0);
  
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  last_odom_loc_ = odom_loc_;
  odom_loc_ = loc;
  odom_angle_ = angle;
  odom_dist_traversed_ += (odom_loc_-last_odom_loc_).norm();
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    return;
  }

}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;
}

void Navigation::estimate_latency_compensated_odometry(Eigen::Vector2f* projected_loc, 
                                                        float* projected_angle,
                                                        Eigen::Vector2f* projected_vel,
                                                        float* projected_dist_traversed) {
  for(uint i = 1; i<vel_history_.size(); ++i) {
      if (((vel_history_[i]-vel_history_[i-1])/DT) > MAX_ACCELERATION || ((vel_history_[i]-vel_history_[i-1])/DT) < -MAX_DECELERATION) {
        vel_history_[i] = std::min(MAX_VELOCITY, vel_history_[i-1] + MAX_ACCELERATION*DT);
      }
      *projected_dist_traversed += (vel_history_[i-1] + vel_history_[i])*(DT/2);
      *projected_vel = Vector2f(cos(steer_history_[i]), sin(steer_history_[i]))*vel_history_[i];
      *projected_loc += *projected_vel * DT;
      *projected_angle += (vel_history_[i]/WHEELBASE)*tan(steer_history_[i])*DT;
  }
}

float Navigation::compute_toc(float distance_to_target, float init_v) {
  float t1 = (MAX_VELOCITY-init_v)/MAX_ACCELERATION;
  float x1 = 0.5*(init_v+MAX_VELOCITY)*t1;
  float x3 = (MAX_VELOCITY*MAX_VELOCITY)/(2*MAX_DECELERATION);
  float x2 = distance_to_target - x1 - x3;
  float t2 = x2/MAX_VELOCITY;
  if (t2 < DT) { // Not enough time to accelerate to cruising speed
    float new_x3 = (init_v*init_v)/(2*MAX_DECELERATION);
    float new_x1 = distance_to_target - new_x3;
    if (new_x1 < 0) { // Not enough distance to brake
      return 0.0; 
    }
    float target_v = std::min(MAX_VELOCITY, sqrt(2*MAX_ACCELERATION*new_x1));
    return target_v;
  } else { // Enough time to accelerate to cruising speed
    float target_v = std::min(MAX_VELOCITY, init_v+MAX_ACCELERATION*DT);
    return target_v;
  }

  // float next_velocity = (init_v) + MAX_ACCELERATION*DT;
  // float time_to_reach_target = distance_to_target/next_velocity;
  // float time_to_decelerate = next_velocity/MAX_DECELERATION;
  // if(time_to_reach_target < time_to_decelerate)
  // {
  //   return 0.0;
  // }
  // else{
  //   return MAX_VELOCITY;
  // }

}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 

  // STEP 1: Latency compensation-odometry  
  // Project our position to estimated position actuation latency from now.
  // Use history of actuations 1 system latency ago
  Vector2f projected_loc = odom_loc_;
  float projected_angle = odom_angle_;
  Vector2f projected_velocity = robot_vel_;
  float projected_dist_traversed = odom_dist_traversed_;
  estimate_latency_compensated_odometry(&projected_loc, &projected_angle, &projected_velocity, &projected_dist_traversed);

  // STEP 2: Latency compensation-point_cloudd
  // The latest observed point cloud is accessible via "point_cloud_"
  float dangle = odom_angle_ - projected_angle;
  Vector2f dloc = odom_loc_ - projected_loc;
  Rotation2Df rotation(dangle);
  for(uint i = 0; i < point_cloud_.size(); ++i) {
    point_cloud_[i] = (rotation*point_cloud_[i])+dloc;
  }

  // STEP 3,4: Do obstacle avoidance calculations to determine target steering angle/curvature
  // For every steering angle
  for(float theta = MIN_STEER; theta < MAX_STEER; theta+=DSTEER) {
    // For every particle
    if (fabs(theta) < kEpsilon) {
      // Handle special case for going straight
      for(uint i = 0; i < point_cloud_.size(); ++i) {
      }
    } else {
      float curvature = tan(theta)/WHEELBASE;
      float radius = 1/curvature;
      (void) radius; (void) curvature; // Placeholder to compile for now
      for(uint i = 0; i < point_cloud_.size(); ++i) {
      }
    }
  }
  drive_msg_.curvature = 0;

  // STEP 5: Apply 1D TOC
  float distance_to_target = 15-projected_dist_traversed; 
  drive_msg_.velocity = compute_toc(distance_to_target, projected_velocity.norm());


  // STEP 6: Update History
  vel_history_.pop_front();
  steer_history_.pop_front();
  vel_history_.push_back(drive_msg_.velocity);
  steer_history_.push_back(drive_msg_.curvature);

  // std::cout << "velocity history: " << vel_history_ << '\n';
  // std::cout << "steering history: " << steer_history_ << '\n';
  // std::cout << "odom position x: " << odom_loc_.x() << '\n';
  // std::cout << "odom position y: " << odom_loc_.y() << '\n';
  // std::cout << "odom angle: " << odom_angle_ << '\n';
  // std::cout << "projected position x: " << projected_loc.x() << '\n';
  // std::cout << "projected position y: " << projected_loc.y() << '\n';
  // std::cout << "projected angle: " << projected_angle << '\n';
  // std::cout << "nav goal x: " << nav_goal_loc_.x() << '\n';
  // std::cout << "nav goal y: " << nav_goal_loc_.y() << '\n';
  // std::cout << "dist to target: " << distance_to_target << '\n';

  visualization::DrawLine(Vector2f(0, 0), projected_loc-odom_loc_, 0xff0000, local_viz_msg_);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
