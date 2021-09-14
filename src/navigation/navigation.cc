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

DEFINE_double(safety_margin, 0.1, "Saftey margin around robot, in meters");

constexpr double LENGTH = 0.4;
constexpr double WHEEL_BASE = 0.3175;
constexpr double WIDTH = 0.15;
constexpr double TRACK_WIDTH = 0.1;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

inline float RadiusOfPoint(float radius, float x, float y) {
  return sqrt(pow(radius - y, 2) + pow(x, 2));
};
inline float RadiusOfPoint(float radius, Eigen::Vector2f& point) {
  return RadiusOfPoint(radius, point.x(), point.y());
};

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
    front_left_corner_(WHEEL_BASE+0.5*(LENGTH-WHEEL_BASE)+FLAGS_safety_margin, 0.5*WIDTH + FLAGS_safety_margin),
    front_right_corner_(front_left_corner_.x(), -1*front_left_corner_.y()),
    back_right_corner_(-0.5*(LENGTH-WHEEL_BASE)-FLAGS_safety_margin, front_right_corner_.y()),
    back_left_corner_(back_right_corner_.x(), front_left_corner_.y()),
    left_wheel_outside_(0.0, front_left_corner_.y()),
    right_wheel_outside_(0.0, front_right_corner_.y()),
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
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    last_odom_loc_ = loc;
    odom_loc_ = loc;
    odom_angle_ = angle;
    odom_dist_traversed_ = 0;
    odom_initialized_ = true;
    return;
  }
  odom_angle_ = angle;
  last_odom_loc_ = odom_loc_;
  odom_loc_ = loc;
  odom_dist_traversed_ += (odom_loc_-last_odom_loc_).norm();
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
  // float t1 = (MAX_VELOCITY-init_v)/MAX_ACCELERATION;
  // float x1 = 0.5*(init_v+MAX_VELOCITY)*t1;
  // float x3 = (MAX_VELOCITY*MAX_VELOCITY)/(2*MAX_DECELERATION);
  // float x2 = distance_to_target - x1 - x3;
  // float t2 = x2/MAX_VELOCITY;
  // if (t2 < DT) { // Not enough time to accelerate to cruising speed
  //   float new_x3 = (init_v*init_v)/(2*MAX_DECELERATION);
  //   float new_x1 = distance_to_target - new_x3;
  //   if (new_x1 < 0) { // Not enough distance to brake
  //     return 0.0; 
  //   }
  //   float target_v = std::min(MAX_VELOCITY, sqrt(2*MAX_ACCELERATION*new_x1));
  //   return target_v;
  // } else { // Enough time to accelerate to cruising speed
  //   float target_v = std::min(MAX_VELOCITY, init_v+MAX_ACCELERATION*DT);
  //   return target_v;
  // }

  float next_velocity = (init_v) + MAX_ACCELERATION*DT;
  float time_to_reach_target = distance_to_target/next_velocity;
  float time_to_decelerate = next_velocity/MAX_DECELERATION;
  if(time_to_reach_target < time_to_decelerate)
  {
    return 0.0;
  }
  else{
    return MAX_VELOCITY;
  }

}

void Navigation::apply_latency_compensated_odometry(Vector2f dloc, float dangle) {
  Rotation2Df rotation(dangle);
  for(uint i = 0; i < point_cloud_.size(); ++i) {
    point_cloud_[i] = (rotation*point_cloud_[i])+dloc;
    visualization::DrawPoint(point_cloud_[i], 0xFF0000, local_viz_msg_);
  }
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

  // STEP 2: Latency compensation-point_cloud/vehicle landmarks
  // The latest observed point cloud is accessible via "point_cloud_"
  float dangle = odom_angle_ - projected_angle;
  Vector2f dloc = odom_loc_ - projected_loc;
  apply_latency_compensated_odometry(dloc, dangle);

  // STEP 3,4: Do obstacle avoidance calculations to determine target steering angle/curvature
  // For every steering angle
  drive_msg_.curvature = 0.0;
  float chosen_free_path_length = 0.0; // Chosen based on value for target below
  float max_weighted_score = 0.0;
  // int number_of_angles = (int) ((MAX_STEER - MIN_STEER) / DSTEER);
  // float min_distance_to_goal[number_of_angles];
  // float free_path_lengths[number_of_angles];
  // float curvatures[number_of_angles]; //save some calculations
  for(float theta = MIN_STEER; theta < MAX_STEER; theta+=DSTEER) {
    float new_free_path_length = 1000.0;
    float curvature = tan(theta)/WHEELBASE;
    float radius = curvature < kEpsilon ? 0 : 1/curvature;
    // For every particle
    if (fabs(theta) < kEpsilon) {
      // Handle special case for going straight
      for(uint i = 0; i < point_cloud_.size(); ++i) {
        if (point_cloud_.at(i).y() >= front_right_corner_.y() && point_cloud_.at(i).y() <= front_left_corner_.y()) {
          // Point will collide
          new_free_path_length = std::min(new_free_path_length, point_cloud_.at(i).x() - front_left_corner_.x());
        }
      }
      // Draw the path
      visualization::DrawLine(Eigen::Vector2f(front_left_corner_.x(), 0.0), Eigen::Vector2f(front_left_corner_.x() + new_free_path_length, 0.0), 0x0000ff, local_viz_msg_);
    } else {
      float max_arc_angle = M_PI;
      for(uint i = 0; i < point_cloud_.size(); ++i) {
        // Check for collision
        Collision collision = CheckCollision(radius, point_cloud_.at(i));

        // Skip if this point won't collide
        if (collision == NONE) {
          // Eventually calculate clearance here?
          new_free_path_length = radius * max_arc_angle;
          continue;
        }

        // Find point on car where it will collide
        Eigen::Vector2f collision_point = GetCollisionPoint(radius, RadiusOfPoint(radius, point_cloud_.at(i)), collision);

        // Get arc-angle to point and collision point
        // Convert to the frame at the turning point: Py' = Py - r
        float angle_to_point = atan2(point_cloud_.at(i).y() - radius, point_cloud_.at(i).x());
        float angle_to_collision = atan2(collision_point.y() - radius, collision_point.x());

        // Do theta_max = angle_to_point - angle_to_collision
        float arc_angle = angle_to_point - angle_to_collision;

        // Fix the angle [0, 2*pi]
        if (arc_angle < 0) {
          arc_angle += 2*M_PI;
        }

        // Track the min
        if (arc_angle < max_arc_angle) {
          max_arc_angle = arc_angle;
          new_free_path_length = radius * arc_angle;
        }
      }
      // Draw the path
      // visualization::DrawPathOption(curvature, new_free_path_length, 0.0, local_viz_msg_);
      // visualization::DrawArc(Eigen::Vector2f(0.0, radius), radius, -M_PI_2, max_arc_angle - M_PI_2, 0x0000ff, local_viz_msg_);
    }
    float min_distance_to_goal = sqrtf32(15*15 + radius*radius) - radius;
    float weighted_score = -1*min_distance_to_goal + new_free_path_length;
    if (weighted_score > max_weighted_score) {
      chosen_free_path_length = new_free_path_length;
      max_weighted_score  = weighted_score;
      drive_msg_.curvature = curvature;
      // std::cout << "weighted_score : " << weighted_score << '\n';
      }
    }
    visualization::DrawPathOption(chosen_curvature, chosen_free_path_length, 0.0, local_viz_msg_);

  // STEP 5: Apply 1D TOC 
  drive_msg_.velocity = compute_toc(chosen_free_path_length, projected_velocity.norm());


  // STEP 6: Update History
  vel_history_.pop_front();
  steer_history_.pop_front();
  vel_history_.push_back(drive_msg_.velocity);
  steer_history_.push_back(atan(drive_msg_.curvature*WHEEL_BASE));

  std::cout << "velocity history: " << vel_history_ << '\n';
  std::cout << "steering history: " << steer_history_ << '\n';
  std::cout << "fpl: " << chosen_free_path_length << '\n';
  std::cout << "Max weighted score: " << max_weighted_score << std::endl;
  // std::cout << "odom position x: " << odom_loc_.x() << '\n';
  // std::cout << "odom position y: " << odom_loc_.y() << '\n';
  // std::cout << "odom angle: " << odom_angle_ << '\n';
  // std::cout << "projected position x: " << projected_loc.x() << '\n';
  // std::cout << "projected position y: " << projected_loc.y() << '\n';
  // std::cout << "projected angle: " << projected_angle << '\n';

  visualization::DrawLine(Vector2f(0, 0), projected_loc-odom_loc_, 0xff0000, local_viz_msg_);

  DrawCar(0xff0000, local_viz_msg_);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

void Navigation::DrawCar(uint32_t color, amrl_msgs::VisualizationMsg& msg) {
  visualization::DrawLine(front_right_corner_, front_left_corner_, color, msg);
  visualization::DrawLine(front_right_corner_, back_right_corner_, color, msg);
  visualization::DrawLine(back_left_corner_, back_right_corner_, color, msg);
  visualization::DrawLine(front_left_corner_, back_left_corner_, color, msg);
}

Collision Navigation::CheckCollision(float radius, Eigen::Vector2f& point) {
  // Handle left and right turns by flipping sign on points of interest
  float car_max_y_value = copysign(front_left_corner_.y(), radius);

  // Get radii for transition points in collision, and pointcloud point
  float radius_P = RadiusOfPoint(radius, point);
  float radius_C = RadiusOfPoint(radius, left_wheel_outside_.x(), car_max_y_value);
  float radius_B = RadiusOfPoint(radius, front_left_corner_.x(), car_max_y_value);
  float radius_A = RadiusOfPoint(radius, front_left_corner_.x(), -1*car_max_y_value);
  float radius_D = RadiusOfPoint(radius, left_wheel_outside_.x(), -1*car_max_y_value);
  float radius_E = RadiusOfPoint(radius, back_right_corner_.x(), -1*car_max_y_value);

  // Check collision criteria
  if (radius_P >= radius_C && radius_P < radius_B) {
    return INSIDE;
  }
  if (radius_P >= radius_B && radius_P <= radius_A) {
    return FRONT;
  }
  if (radius_P >= radius_D && radius_P <= radius_E && point.x() >= back_right_corner_.x() && point.x() <= right_wheel_outside_.x()) {
    return OUTSIDE;
  }

  return NONE;
}

Eigen::Vector2f Navigation::GetCollisionPoint(float turn_radius, float point_radius, Collision collision_type) {
  Eigen::Vector2f output;
  switch(collision_type) {
    case FRONT:
      output.x() = front_left_corner_.x();
      output.y() = turn_radius - sqrt(pow(point_radius, 2) - pow(output.x(), 2));
      return output;
    case INSIDE:
      output.y() = copysign(front_left_corner_.y(), turn_radius);
      output.x() = sqrt(pow(point_radius, 2) - pow(turn_radius - output.y(), 2));
      return output;
    case OUTSIDE:
      output.y() = copysign(front_left_corner_.y(), -1*turn_radius);
      output.x() = sqrt(pow(point_radius, 2) - pow(turn_radius - output.y(), 2));
      return output;
    default:
      throw std::invalid_argument("Invalid collision type");
  }
}

}  // namespace navigation
