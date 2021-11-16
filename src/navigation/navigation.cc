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
#include "shared/math/geometry.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "vector_map/vector_map.h"
#include "graph/graph.h"

using Eigen::Vector2f;
using Eigen::Rotation2Df;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

DEFINE_double(safety_margin, 0.15, "Safety margin around robot, in meters");
DEFINE_double(d2g_weight, 0.05, "Distance to goal weight");
DEFINE_double(fpl_weight, 0.25, "Free path length weight");
DEFINE_double(clearance_weight, .25, "Clearance weight");
DEFINE_bool(verbose, false, "Print some debug info in the control loop");
DEFINE_double(carrot_radius, 5.0, "Max distance for local goal");

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

inline bool checkGoalReached(const Vector2f& goal, const Vector2f& current_loc, float threshold=0.5) {
  return (goal - current_loc).norm() < threshold;
}

bool pointIsCloseToSegment(const Vector2f& point, const Vector2f& seg_a, const Vector2f& seg_b, float threshold=0.5) {
  bool output = false;

  // Perp distance: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
  const float dist_numer = fabs((seg_b.x()-seg_a.x())*(seg_a.y()-point.y()) - (seg_a.x()-point.x())*(seg_b.y()-seg_a.y()));
  const float perp_dist = dist_numer/(seg_a - seg_b).norm();

  // Check 3 cases: perpindicular distance to line, and distance to each end point
  output |= perp_dist < threshold;
  output |= (seg_a - point).norm() < threshold;
  output |= (seg_b - point).norm() < threshold;

  return output;
}

std::map<navigation::Collision, std::string> collision_string_ { {navigation::NONE, "NONE"}, {navigation::FRONT, "FRONT"}, {navigation::INSIDE, "INSIDE"}, {navigation::OUTSIDE, "OUTSIDE"}, };
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
    nav_set_(false),
    nav_complete_(false),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    front_left_corner_(WHEELBASE+0.5*(LENGTH-WHEELBASE)+FLAGS_safety_margin, 0.5*WIDTH + FLAGS_safety_margin),
    front_right_corner_(front_left_corner_.x(), -1*front_left_corner_.y()),
    back_right_corner_(-0.5*(LENGTH-WHEELBASE)-FLAGS_safety_margin, front_right_corner_.y()),
    back_left_corner_(back_right_corner_.x(), front_left_corner_.y()),
    left_wheel_outside_(0.0, front_left_corner_.y()),
    right_wheel_outside_(0.0, front_right_corner_.y()),
    system_latency_(system_latency),
    act_latency_((ac_to_obs/(ac_to_obs+1.0))*system_latency),
    obs_latency_((1.0/(ac_to_obs+1.0))*system_latency),
    map_(map_file),
    graph_(map_) {
  
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

void Navigation::GlobalPlan() {
  global_plan_ = graph_.ShortestPath(robot_loc_, nav_goal_loc_);
  for(auto& p: global_plan_) {
    printf("(%f, %f)\n", p.x(), p.y());
  }
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = loc;
  (void) angle;
  nav_set_ = true;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
  printf("%d\n", nav_set_);
  if (nav_set_) {
    GlobalPlan();
  }
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
      if (((vel_history_[i]-vel_history_[i-1])/DT) > MAX_ACCELERATION) {
        vel_history_[i] = std::min(MAX_VELOCITY, vel_history_[i-1] + MAX_ACCELERATION*DT);
      } else if (((vel_history_[i]-vel_history_[i-1])/DT) < -MAX_DECELERATION) {
        vel_history_[i] = std::max(0.0, vel_history_[i-1] -MAX_DECELERATION*DT);
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
}

void Navigation::apply_latency_compensated_odometry(Vector2f dloc, float dangle) {
  Rotation2Df rotation(dangle);
  for(uint i = 0; i < point_cloud_.size(); ++i) {
    point_cloud_[i] = (rotation*point_cloud_[i])+dloc;
    // visualization::DrawPoint(point_cloud_[i], 0xFF0000, local_viz_msg_);
  }
}

float Navigation::compute_arc_distance_to_goal(float arc_radius, Eigen::Vector2f goal, bool straight=false, float max_angle=M_PI)
{
  if (!straight) {
    // float angle_to_goal = std::min(atan2(goal.x(), goal.y() - arc_radius), max_angle);
    // float delta_x = arc_radius*sin(angle_to_goal) - goal.x();
    // float delta_y = arc_radius*cos(angle_to_goal) - goal.y();
    // return sqrt(delta_x*delta_x + delta_y*delta_y);
     auto goal2 = Eigen::Vector2f(goal.x() + 0.01f, goal.y() + 0.01f);
     auto center = Eigen::Vector2f(0.0f, arc_radius);
     float min_a_angle = arc_radius > 0 ? (float) (-M_PI/2) : (float) (-max_angle + M_PI/2);
     float max_a_angle = arc_radius > 0 ? (float) (-M_PI/2 + max_angle): (float) (0.0f + M_PI/2);
    return geometry::MinDistanceLineArc(goal, goal2, center, fabs(arc_radius), min_a_angle, max_a_angle, 1);
  } else {
    if (goal.x() < 0) {
      return goal.norm();
    } else {
      Eigen::Vector2f distance(std::max(0.0F, goal.x() - max_angle), goal.y());
      return distance.norm();
    }
  }
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_ || !localization_initialized_) return;

  // Always publish the car visualization
  DrawCar(0xff0000, local_viz_msg_);
  local_viz_msg_.header.stamp = ros::Time::now();

  // The control iteration goes here. 

  // STEP 1: Latency compensation-odometry  
  // Project our position to estimated position actuation latency from now.
  // Use history of actuations 1 system latency ago
  Vector2f projected_loc = odom_loc_;
  float projected_angle = odom_angle_;
  Vector2f projected_velocity = robot_vel_;
  float projected_dist_traversed = odom_dist_traversed_;
  estimate_latency_compensated_odometry(&projected_loc, &projected_angle, &projected_velocity, &projected_dist_traversed);

  // If there is no plan or we finished it, don't do anything
  // Note: this assumes the last element of the global plan is the goal
  if (global_plan_.empty() || checkGoalReached(global_plan_.back(), robot_loc_ + (Eigen::Rotation2Df(projected_angle - odom_angle_) * (projected_loc - odom_loc_)), 0.5)) return;

  // Check if we need to replan
  std::pair<bool, Eigen::Vector2f> local_goal = getLocalGoal();
  if (!local_goal.first) {
    GlobalPlan();
    return;
  }
  auto goal = local_goal.second;
  visualization::DrawCross(goal, 1.5, 0x34eb49, local_viz_msg_);
  viz_pub_.publish(local_viz_msg_);

  // Visualize global plan
  for (size_t i=0 ; i < global_plan_.size()-1; i++) {
    visualization::DrawLine(global_plan_.at(i), global_plan_.at(i+1), 0x203ee8, global_viz_msg_);
  }


  // STEP 2: Latency compensation-point_cloud/vehicle landmarks
  // The latest observed point cloud is accessible via "point_cloud_"
  float dangle = odom_angle_ - projected_angle;
  Vector2f dloc = odom_loc_ - projected_loc;
  apply_latency_compensated_odometry(dloc, dangle);

  // STEP 3,4: Do obstacle avoidance calculations to determine target steering angle/curvature
  // For every steering angle
  drive_msg_.curvature = 0.0;
  float chosen_free_path_length = 0.0;
  float chosen_curvature = 0.0;
  (void) chosen_curvature;
  float max_weighted_score = 0.0;
  float chosen_distance_to_goal = 0.0;
  std::map<int, PathOption> path_options;
  int loop_counter = 0;
  float absolute_min_distance2goal = 100000.0f;
  for(float theta = math_util::DegToRad(MIN_STEER); theta < math_util::DegToRad(MAX_STEER); theta+=math_util::DegToRad(DSTEER)) {
    float new_free_path_length = 10.0;
    float curvature = tan(theta)/WHEELBASE;
    float radius = fabs(curvature) < kEpsilon ? INT_MAX: 1/curvature;
    path_options[loop_counter] = PathOption();
    // For every particle
    if (fabs(theta) < kEpsilon) {
      // Handle special case for going straight
      path_options.at(loop_counter).curvature = 0;
      path_options.at(loop_counter).collision_type = NONE;
      for(uint i = 0; i < point_cloud_.size(); ++i) {
        if (point_cloud_.at(i).y() >= front_right_corner_.y() && point_cloud_.at(i).y() <= front_left_corner_.y()) {
          // Point will collide
          if (new_free_path_length > point_cloud_.at(i).x() - front_left_corner_.x()) {
            new_free_path_length = point_cloud_.at(i).x() - front_left_corner_.x();
            path_options.at(loop_counter).free_path_length = new_free_path_length;
            path_options.at(loop_counter).collision_type = FRONT;
            path_options.at(loop_counter).obstruction = point_cloud_.at(i);
            path_options.at(loop_counter).closest_point = Eigen::Vector2f(front_left_corner_.x(), point_cloud_.at(i).y());
          }
        }
      }
      path_options.at(loop_counter).min_distance_to_goal = compute_arc_distance_to_goal(radius, goal, true, new_free_path_length); 
      absolute_min_distance2goal = std::min(absolute_min_distance2goal, path_options.at(loop_counter).min_distance_to_goal);
      // Draw the path
      visualization::DrawLine(Eigen::Vector2f(front_left_corner_.x(), 0.0), Eigen::Vector2f(front_left_corner_.x() + new_free_path_length, 0.0), 0x0000ff, local_viz_msg_);
    } else {
      float max_arc_angle = M_PI;
      path_options.at(loop_counter).curvature = curvature;
      path_options.at(loop_counter).collision_type = NONE;
      new_free_path_length = fabs(radius * max_arc_angle);
      for(uint i = 0; i < point_cloud_.size(); ++i) {
        // Check for collision
        Collision collision = CheckCollision(radius, point_cloud_.at(i));

        // Skip if this point won't collide
        if (collision == NONE) {
          continue;
        }
        // Find point on car where it will collide
        Eigen::Vector2f collision_point = GetCollisionPoint(radius, RadiusOfPoint(radius, point_cloud_.at(i)), collision);

        // Get arc-angle to point and collision point
        // Convert to the frame at the turning point: Py' = Py - r
        float angle_to_point = atan2(point_cloud_.at(i).y() - radius, point_cloud_.at(i).x());
        float angle_to_collision = atan2(collision_point.y() - radius, collision_point.x());

        // Do theta_max = angle_to_point - angle_to_collision
        float arc_angle;
        if (radius > 0) {
          arc_angle = angle_to_point - angle_to_collision;
        } else {
          arc_angle = angle_to_collision - angle_to_point;
        }

        // Fix the angle [0, 2*pi]
        if (arc_angle < 0) {
          arc_angle += 2*M_PI;
        }        

        // Check if this particle causes the shortest travel distance
        if (arc_angle < max_arc_angle) {
          max_arc_angle = arc_angle;
          new_free_path_length = fabs(radius * arc_angle);
          path_options.at(loop_counter).collision_type = collision;
          path_options.at(loop_counter).closest_point = collision_point;
          path_options.at(loop_counter).obstruction = point_cloud_.at(i);
        }
      }
      path_options.at(loop_counter).free_path_length = new_free_path_length;
      path_options.at(loop_counter).min_distance_to_goal = compute_arc_distance_to_goal(radius, goal, false, max_arc_angle);
      absolute_min_distance2goal = std::min(absolute_min_distance2goal, path_options.at(loop_counter).min_distance_to_goal);
      // Draw the path
      visualization::DrawCross(path_options.at(loop_counter).obstruction, 0.1, 0xff0000, local_viz_msg_);
      visualization::DrawCross(path_options.at(loop_counter).closest_point, 0.1, 0x0000ff, local_viz_msg_);
      visualization::DrawPathOption(curvature, new_free_path_length, 0.0, local_viz_msg_);
    }
    // since the current theta is responsible for the previous theta's clearance, we can't run this with the first theta
    if(loop_counter >= 1)
    {
      float previous_path_clearance = path_options.at(loop_counter).free_path_length; // if we have three free path lengths, 0 1 2, then 0's clearance = avg(1), 1's clearance = avg(0,2), and 2's clearance = avg(1)
      float included_paths = 1;
      // are we at the third iteration in our loop? (i.e., are there currently three existing free path lengths that we can index into?)
      if(loop_counter >= 2)
      {
        previous_path_clearance += path_options.at(loop_counter - 2).free_path_length;
        included_paths += 1;
      }
      // The idea of "clearance" implemented here was inspired by discussions with other team in the class (Johnny 6)
      path_options.at(loop_counter - 1).clearance = previous_path_clearance/included_paths; //set the clearance of the previous path as the average itself and its neighboring paths
      path_options.at(loop_counter - 1).score += FLAGS_clearance_weight*path_options.at(loop_counter - 1).clearance; //also adjust their score
    }

    path_options.at(loop_counter).score = -FLAGS_d2g_weight*path_options.at(loop_counter).min_distance_to_goal + FLAGS_fpl_weight*path_options.at(loop_counter).free_path_length;;
    float current_score = path_options.at(loop_counter).score;
    int max_index = loop_counter;
    if(loop_counter >= 1)
    {
      if(current_score < path_options.at(loop_counter-1).score)
      {
        //compare with the previous score as well
        current_score = path_options.at(loop_counter-1).score;
        max_index = loop_counter -1;
      }

    }
    if (current_score > max_weighted_score) {
      chosen_free_path_length = path_options.at(max_index).free_path_length;
      max_weighted_score  = path_options.at(max_index).score;
      chosen_curvature = path_options.at(max_index).curvature;
      chosen_distance_to_goal = path_options.at(max_index).min_distance_to_goal;
      }
     ++loop_counter;
  }
  // Since clearance for a radius r is set in the r+1 iteration, the final radius will not have its clearance set in this loop. The following logic remedies this issue
  path_options.at(loop_counter-1).clearance = path_options.at(loop_counter-2).free_path_length;
  path_options.at(loop_counter-1).score += -FLAGS_d2g_weight*path_options.at(loop_counter-1).clearance;
  if (path_options.at(loop_counter-1).score > max_weighted_score) {
        chosen_free_path_length = path_options.at(loop_counter-1).free_path_length;
        max_weighted_score  = path_options.at(loop_counter-1).score;
        chosen_curvature = path_options.at(loop_counter-1).curvature;
        chosen_distance_to_goal = path_options.at(loop_counter-1).min_distance_to_goal;
  }


  if ((robot_loc_-global_plan_.back()).norm() <= 4.0f)
  {
    std::cout << "orig fpl: " << chosen_free_path_length << '\n';
    std::cout << "orig curv: " << chosen_curvature << '\n';
    std::cout << "orig mws: " << max_weighted_score << '\n';
    for (auto& po: path_options)
    {
      printf("Curvature (%f) min dist to goal (%f)\n", po.second.curvature, po.second.min_distance_to_goal);
      if (fabs(po.second.min_distance_to_goal - absolute_min_distance2goal) <= 1e-3)
      {
        chosen_free_path_length = po.second.free_path_length;
        max_weighted_score  = po.second.score;
        chosen_curvature = po.second.curvature;
        chosen_distance_to_goal = po.second.min_distance_to_goal;
      }
    }
    std::cout << "new fpl: " << chosen_free_path_length << '\n';
    std::cout << "new curv: " << chosen_curvature << '\n';
    std::cout << "new mws: " << max_weighted_score << '\n';
  }

  visualization::DrawPathOption(chosen_curvature, chosen_free_path_length, 0.0, local_viz_msg_);
  drive_msg_.curvature = chosen_curvature;
  // (void) chosen_curvature;
  // drive_msg_.curvature = 1.0;
  // STEP 5: Apply 1D TOC to determine velocity 
// STEP 5: Apply 1D TOC to determine velocity 
  // STEP 5: Apply 1D TOC to determine velocity 
  drive_msg_.velocity = compute_toc(chosen_free_path_length, projected_velocity.norm());
  // drive_msg_.velocity = 0.3;


  // STEP 6: Update History
  vel_history_.pop_front();
  steer_history_.pop_front();
  vel_history_.push_back(drive_msg_.velocity);
  steer_history_.push_back(atan(drive_msg_.curvature*WHEELBASE));

  if (FLAGS_verbose) {
    std::cout << "velocity history: " << vel_history_ << '\n';
    std::cout << "steering history: " << steer_history_ << '\n';
    std::cout << "fpl: " << chosen_free_path_length << '\n';
    std::cout << "Max weighted score: " << max_weighted_score << std::endl;
    std::cout << "chosen distance to target: " << chosen_distance_to_goal << std::endl;
  }

  // Add timestamps to all messages.
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(global_viz_msg_);
  viz_pub_.publish(local_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

void Navigation::DrawCar(uint32_t color, amrl_msgs::VisualizationMsg& msg) {
  visualization::DrawLine(front_right_corner_, front_left_corner_, color, msg);
  visualization::DrawLine(front_right_corner_, back_right_corner_, color, msg);
  visualization::DrawLine(back_left_corner_, back_right_corner_, color, msg);
  visualization::DrawLine(front_left_corner_, back_left_corner_, color, msg);
  visualization::DrawArc(Vector2f(0,0), FLAGS_carrot_radius, 0, 2*M_PI, 0xe834eb, msg);
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

  if (collision_type == FRONT) {
    output.x() = front_left_corner_.x();
    
    // We must check both +- on the square root
    float point_option1 = turn_radius - sqrt(pow(point_radius, 2) - pow(output.x(), 2));
    if (IsBetween(-front_left_corner_.y(), point_option1, front_left_corner_.y())) {
      output.y() = point_option1;
      return output;
    }
    float point_option2 = turn_radius + sqrt(pow(point_radius, 2) - pow(output.x(), 2));
    if (IsBetween(-front_left_corner_.y(), point_option2, front_left_corner_.y())) {
      output.y() = point_option2;
      return output;
    } else {
      std::cout << "Could not find collision point for front between " << -front_left_corner_.y() << " and " << front_left_corner_.y() << std::endl;
      std::cout << "Point option 1 " << point_option1 << " and Point option 2 " << point_option2 << std::endl;
      return output;
    }
  }

  if (collision_type == INSIDE) {
    output.y() = copysign(front_left_corner_.y(), turn_radius);
  }
  else if (collision_type == OUTSIDE) {
    output.y() = copysign(front_left_corner_.y(), -1*turn_radius);
  } else {
    throw std::invalid_argument("Invalid collision type");
  }

  float point_option1 = sqrt(pow(point_radius, 2) - pow(turn_radius - output.y(), 2));
  float point_option2 = -point_option1;
  if (IsBetween(back_left_corner_.x(), point_option1, front_left_corner_.x())) {
    output.x() = point_option1;
  }
  else if (IsBetween(back_left_corner_.x(), point_option2, front_left_corner_.x())) {
    output.x() = point_option2;
  } else {
      std::cout << "Could not find collision point for front between " << back_left_corner_.x() << " and " << front_left_corner_.x() << std::endl;
      std::cout << "Point option 1 " << point_option1 << " and Point option 2 " << point_option2 << std::endl;
      return output;
  }

  return output;
}

// Returns goal in robot's local frame
std::pair<bool, Eigen::Vector2f> Navigation::getLocalGoal() {
  auto output = std::make_pair(false, Vector2f(0, 0));

  // If we are close enough to the goal, set it directly
  if ((global_plan_.back() - robot_loc_).norm() < FLAGS_carrot_radius) {
    output.first = true;
    output.second = Eigen::Rotation2Df(-1*robot_angle_) * (global_plan_.back() - robot_loc_);
    visualization::DrawCross(global_plan_.back(), 5, 0x000000, global_viz_msg_);
    return output;
  }

  // Go backwards through the global plan and check if we are close to the segment
  bool found_local_goal = false;
  for(size_t i = global_plan_.size() - 1; i >= 1; i--) {
    // Check if we are close to this segment of the plan
    if (pointIsCloseToSegment(robot_loc_, global_plan_.at(i), global_plan_.at(i-1), 0.5)) {
      output.first = true;
    }

    // Check if this segment intersects with the carrot circle
    // If already found, continue (set it to segment intersect closest to goal)
    if (output.first && found_local_goal) return output;
    if (found_local_goal) continue;

    // Vector2f intersection;
    // if (segmentIntersectsCircle(global_plan_.at(i), global_plan_.at(i-1), robot_loc_, FLAGS_carrot_radius, &intersection)) {
    //   found_local_goal = true;
    //   visualization::DrawCross(intersection, 5, 0x000000, global_viz_msg_);
    //   output.second = Eigen::Rotation2Df(-1*robot_angle_) * (intersection - robot_loc_);
    // }
    if ((global_plan_.at(i) - robot_loc_).norm() < FLAGS_carrot_radius) {
      found_local_goal = true;
      visualization::DrawCross(global_plan_.at(i), 5, 0x000000, global_viz_msg_);
      output.second = Eigen::Rotation2Df(-1*robot_angle_) * (global_plan_.at(i) - robot_loc_);
    }
  }

  return output;
}

}  // namespace navigation
