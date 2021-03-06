#include <algorithm>
#include <stack>
#include <limits>
#include <stdlib.h>

#include "rrt.h"

#include "math.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/util/random.h"
#include "shared/math/math_util.h"
#include "shared/math/line2d.h"
#include "vector_map/vector_map.h"
#include "visualization/visualization.h"
#include "navigation/navigation.h"
#include "ros/ros.h"
#include "shared/ros/ros_helpers.h"

using Eigen::Vector2f;
using Eigen::Rotation2Df;
using amrl_msgs::VisualizationMsg;
using namespace ros_helpers;

using geometry::line2f;

namespace rrt {
VisualizationMsg global_viz_msg_;
ros::Publisher viz_pub_;

Vector2f sampleCircle(util_random::Random& rng, float radius=1.0) {
  const float theta = rng.UniformRandom(0, 2*M_PI);
  const float r = rng.UniformRandom(0, radius);

  return Vector2f(r*cos(theta), r*sin(theta));
}

Vector2f sampleEllipse(util_random::Random& rng, Ellipse& ellipse, double c_best) {
  if (math_util::definitelyGreaterThan(ellipse.c_min, c_best, 1e-7)) {
    char *issue;
    int result =  asprintf(&issue, "c_best (%f) can't be smaller than c_min (%f)!", c_best, ellipse.c_min);
    (void) result;
    throw std::invalid_argument(issue);
  }

  // See paper, construct L matrix from ellipse axes
  Eigen::Matrix2f L;
  L << 0.5*c_best, 0, 0, 0.5*sqrt(c_best*c_best - ellipse.c_min*ellipse.c_min);

  // Sample from unit ball
  const Vector2f x_ball = sampleCircle(rng, 1.0);

  // Transform sampled point
  return ellipse.rotation * L * x_ball + ellipse.centre;
}

Ellipse::Ellipse(const Eigen::Vector2f& start_point, const Eigen::Vector2f& goal_point) {
  start = start_point;
  goal = goal_point;

  // Do all math for ellipses once
  const Vector2f distance = goal - start;
  // cout << "start " << start << std::endl;
  // cout << "goal " << goal << std::endl;
  c_min = distance.norm() - GOAL_RADIUS;
  centre = start + 0.5*(distance);
  const float theta = atan2(distance.y(), distance.x());
  rotation = Rotation2Df(theta);
}

State::State(){}
State::State(const Eigen::Vector2f& loc, double heading):
  loc(loc), heading(heading) {}

TreeNode::TreeNode(): parent(nullptr), state(), children(), cost(0.0) {}
TreeNode::TreeNode(const Eigen::Vector2f& loc, double heading, TreeNode* parent):
  parent(parent),
  state(loc, heading),
  children(),
  cost(0.0f) {}

TreeNode* TreeNode::AddNewChild(State& child, double cost, std::pair<double, double> control_data)
{
  TreeNode* newChild = new TreeNode(child.loc, child.heading, this);
  newChild->cost = cost;
  children[newChild] = control_data;
  return newChild;
}

void TreeNode::AddExistingChild(TreeNode* newChild, std::pair<double, double> control_data)
{
  children[newChild] = control_data;
}

void TreeNode::RemoveChild(TreeNode* childToRemove)
{
  children.erase(childToRemove);
}


RRT::RRT(Vector2f x_start_loc, double x_start_heading, Vector2f x_goal_loc, double x_goal_heading, std::pair<double, double> x_bounds, std::pair<double, double> y_bounds, const vector_map::VectorMap& map, const VisualizationMsg& map_viz_msg):
  x_start_(x_start_loc),
  x_start_heading_(x_start_heading),
  x_goal_(x_goal_loc),
  x_goal_heading_(x_goal_heading),
  rng_(rand()),
  x_bounds_(x_bounds),
  y_bounds_(y_bounds),
  map_(map),
  ellipse_(x_start_loc, x_goal_loc),
  root_(new TreeNode(x_start_loc, x_start_heading)),
  map_viz_msg_(map_viz_msg) {
      node_ptrs_.push_back(root_);
      ros::NodeHandle n;
      global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
        viz_pub_ = n.advertise<VisualizationMsg>("visualization", 1);

  }

RRT::~RRT() {
  for (auto p: node_ptrs_)
  {
    // printf("Address to delete %p\n", (void *)p);  
    delete p;
  }
}

Vector2f RRT::Sample(double c_max) {
  // If c_max is infinity, return a random sample from whole domain
  // TODO make sure this isinf() works
  if (isinf(c_max)) {
    const double x_rand = rng_.UniformRandom(x_bounds_.first, x_bounds_.second);
    const double y_rand = rng_.UniformRandom(y_bounds_.first, y_bounds_.second);
    return Vector2f(x_rand, y_rand);
  }

  // Normally we just sample from the ellipse
  return sampleEllipse(rng_, ellipse_, c_max);
}

TreeNode* RRT::Nearest(Eigen::Vector2f& x_rand) {
  // TODO
  double minDistance = 100000000.0;
  TreeNode* nearest = nullptr;
  for (auto currentNode: node_ptrs_)
  {
    if ((currentNode->state.loc - x_rand).norm() < minDistance)
    {
      minDistance = (currentNode->state.loc - x_rand).norm();
      nearest = currentNode;
    }
  }
  if (nearest == nullptr)
  {
    printf("Min Distance (%f) node count: (%ld) x_rand: \n", minDistance, node_ptrs_.size());
    cout << x_rand << std::endl;
    cout << ellipse_.centre << std::endl;
    // cout << ellipse_.rotation << std::endl;
  }
  return nearest;
}

State RRT::Steer(State& x_nearest, Eigen::Vector2f& x_rand, double* curvature, double* travel) {
  // Distance to move
  // TODO fill this in
  const double max_dist = 0.1;

  Rotation2Df rot_matrix(x_nearest.heading);

  State best_state;
  double best_goal_distance = std::numeric_limits<double>::infinity();

  // Iterate through the possible steering angles
  // TODO fill turning_radii_ in somewhere
  for(double theta = math_util::DegToRad(MIN_STEER); theta < math_util::DegToRad(MAX_STEER); theta+=math_util::DegToRad(DSTEER)) {
    float curv = tan(theta)/WHEELBASE;
    float radius = fabs(curv) < 1e-5 ? INT_MAX: 1.0f/curv;
    // Check for r = infinity case
    if (fabs(curv) < 1e-5) {
      // Find line to goal
      const Vector2f nearest_to_goal = rot_matrix.inverse() * (x_rand - x_nearest.loc); // x_nearest frame

      // Clamp travel distance to goal
      double travel_distance = nearest_to_goal.x();
      travel_distance = math_util::Clamp(travel_distance, -1*max_dist, max_dist);

      // Find next state
      State new_state;
      new_state.heading = x_nearest.heading;
      new_state.loc = x_nearest.loc + rot_matrix * Vector2f(travel_distance, 0); // Map frame

      // Measure the distance
      const double dist_squared = (new_state.loc - x_rand).squaredNorm();

      // Check if its the best
      if (dist_squared < best_goal_distance) {
        best_goal_distance = dist_squared;
        best_state = new_state;
        *curvature = 0;
        *travel = travel_distance;
      }

      continue;
    }

    // Figure out where center of turn is
    const Vector2f radius_line = rot_matrix * Vector2f(0, radius); // In map frame
    const Vector2f turn_center = x_nearest.loc + radius_line; // In map frame

    // Calculate max alpha for max_dist
    const double max_alpha = fabs(max_dist * curv);

    // Calculate alpha to goal
    const Vector2f center_to_goal = x_rand - turn_center;
    const double goal_direction_sign = (rot_matrix * Vector2f(1,0)).dot(center_to_goal); 
    double alpha = copysign(acos((-1*radius_line.dot(center_to_goal))/(fabs(radius) * center_to_goal.norm())), goal_direction_sign); // Using dot product

    // Set alpha = min(max_alpha, alpha to goal)
    alpha = math_util::Clamp(alpha, -1*max_alpha, max_alpha);

    // Find State after following this radius for alpha
    State new_state;
    new_state.loc = turn_center + Rotation2Df(alpha) * -radius_line;
    new_state.heading = x_nearest.heading + alpha;

    // Measure distance to goal at this State
    const double dist_squared = (new_state.loc - x_rand).squaredNorm();

    // Check if its the best
    if (dist_squared < best_goal_distance) {
      best_goal_distance = dist_squared;
      best_state = new_state;
      *curvature = curv;
      *travel = radius*alpha;
    }
  }

  return best_state;
}

State RRT::SteerLinear(State& x_nearest, Eigen::Vector2f& x_rand) {
  const float max_dist = 0.5;

  const Vector2f line_to_goal = x_rand - x_nearest.loc;

  const double scale = std::min(max_dist, line_to_goal.norm());
  const Vector2f scaled_line_to_goal = scale * line_to_goal / line_to_goal.norm();

  State output;
  output.loc = x_nearest.loc + scaled_line_to_goal;
  output.heading = math_util::AngleMod(atan2(scaled_line_to_goal.y(), scaled_line_to_goal.x()));
  return output;
}

std::vector<TreeNode*> RRT::Near(State& x_new, double neighborhood_radius) {
  std::vector<TreeNode*> neighborhood;
  std::stack<TreeNode*> s;
  for (auto currentNode : node_ptrs_)
  {
    // Maybe we also needd to check if we can actually steer from this location to x_new
    if ((currentNode->state.loc - x_new.loc).norm() < neighborhood_radius)
    {
      neighborhood.push_back(currentNode);
    }
  }
  return neighborhood;
}

bool RRT::CollisionFree(State& x_nearest, double curvature, double distance, std::vector<Vector2f> local_observation_points)
{
  Rotation2Df rot_matrix(x_nearest.heading);

  // Check the map
  // Check for special case: curvature = 0
  if (fabs(curvature) < 1e-5) {
    const float extra_distance = 0.4;
    const Vector2f heading_line = x_nearest.loc + rot_matrix * Vector2f(distance + extra_distance, 0); // Map frame

    for (const auto& line : map_.lines) {
      if (geometry::MinDistanceLineLine(x_nearest.loc, heading_line, line.p0, line.p1) <= 0.4) return false;
    }
    for (const auto& point: local_observation_points)
    {
      Eigen::Vector2f dpoint = point + Eigen::Vector2f(0.01, 0.01);
      if (geometry::MinDistanceLineLine(x_nearest.loc, heading_line, point, dpoint) <= 0.4) return false;
    }
    return true;
  }

  // For arcs
  // Find turn center
  const Vector2f radius_line = rot_matrix * Vector2f(0, 1/curvature); // In map frame
  const Vector2f turn_center = x_nearest.loc + radius_line; // In map frame

  // Find starting angle
  const float start_angle = atan2(-radius_line.y(), -radius_line.x());
  const float delta_angle = distance * curvature;
  const float radius = 1/curvature;
  for (const auto& line : map_.lines) {
    // Check if the arc touches this line
    const float distance_to_arc = geometry::MinDistanceLineArc(line.p0, line.p1, turn_center, radius, start_angle, start_angle + delta_angle, int(copysign(1, delta_angle)));
    if (fabs(distance_to_arc) < 0.4) return false;
  }
  for (const auto& point: local_observation_points) {
    Eigen::Vector2f dpoint = point + Eigen::Vector2f(0.01, 0.01);
    const float distance_to_arc = geometry::MinDistanceLineArc(point, dpoint, turn_center, radius, start_angle, start_angle + delta_angle, int(copysign(1, delta_angle)));
    if (fabs(distance_to_arc) < 0.4) return false;    
  }
  return true;
} 

bool RRT::CollisionFreeLinear(State& x_nearest, State& x_new, std::vector<Vector2f> local_observation_points)
{
  // Check the map
  for (const auto& line : map_.lines) {
    if (geometry::MinDistanceLineLine(x_nearest.loc, x_new.loc, line.p0, line.p1) <= 0.4) return false;
    // if (geometry::MinDistanceLineLine(x_nearest.loc, x_new.loc, line.p0, line.p1) <= 0.05) return false;
  }
  // Check point cloud
  for (const auto& point: local_observation_points)
  {
    Eigen::Vector2f dpoint = point + Eigen::Vector2f(0.01, 0.01);
    if (geometry::MinDistanceLineLine(x_nearest.loc, x_new.loc, point, dpoint) <= 0.4) return false;
    // if (geometry::MinDistanceLineLine(x_nearest.loc, x_new.loc, point, dpoint) <= 0.05) return false;
  }
  return true;
} 

std::vector<std::pair<double, Vector2f>> RRT::KinodynamicInformedRRT(std::vector<Eigen::Vector2f>& points, int max_iterations, double costGap, double optimalCost, double improvement_iterations)
{
  visualization::ClearVisualizationMsg(global_viz_msg_);
  global_viz_msg_ = map_viz_msg_;
  cout << "Planning with Kinodynamic Informed RRT\n" << std::endl;
  // Convert pointcloud to Map frame
  getMapPointCloud(points);

  visualization::DrawCross(x_start_, 0.3, 0x0000FF, global_viz_msg_);
  visualization::DrawCross(x_goal_, 0.3, 0x0000FF, global_viz_msg_);
  std::map<TreeNode*, double> goalNodes;
  TreeNode* x_best = nullptr;
  double c_best = std::numeric_limits<double>::infinity();
  bool found_init_solution = false;
  int num_improve_iter = 0;
  for (int i = 0; i < max_iterations; ++i)
  {
    if (costGap > 0 && (c_best-optimalCost)/optimalCost < costGap)
      break;
    for(auto gn: goalNodes)
    {
      if (gn.second < c_best)
      {
        printf("new c_best (%f)\n", gn.second);
        found_init_solution = true;
        c_best = gn.second;
        x_best = gn.first;
        c_best_overall = c_best;
      }
    }

    if (found_init_solution)
      num_improve_iter++;
    if (improvement_iterations > 0 && improvement_iterations < num_improve_iter)
      break;

    Vector2f x_rand = Sample(c_best);
    TreeNode* x_nearest = Nearest(x_rand);
    if (x_nearest == nullptr)
    {
      printf("xnearest is null\n");
      continue;
    }
    double curvature;
    double distance;
    State x_new = Steer(x_nearest->state, x_rand, &curvature, &distance);
    if (CollisionFree(x_nearest->state, curvature, distance, map_cloud_))
    {
      std::vector<TreeNode*> x_near = Near(x_new, 1.0f);
      TreeNode* x_min = x_nearest;
      double c_min = x_nearest->cost + fabs(distance);
      double curvature_min = curvature;
      double distance_min = distance;
      for(auto other_near: x_near)
      {
        //printf("Other near %p\n", other_near);
        // Check kinematic feasibility of going from other_near to x_new
        State other_x_new = Steer(other_near->state, x_new.loc, &curvature, &distance);
        double c_new = other_near->cost + fabs(distance);
        // check if new cost is smaller and the closest steering input is actuall close to the desired x_new
        if (c_new < c_min && (other_x_new.loc - x_new.loc).norm() <= 1e-4)
        {
          if (CollisionFree(other_near->state, curvature, distance, map_cloud_))
          {
            x_min = other_near;
            c_min = c_new;
            curvature_min = curvature;
            distance_min = distance;
          }
        }
      }
      TreeNode* added_x_new = x_min->AddNewChild(x_new, c_min, std::make_pair(curvature_min, distance_min));
      //printf("Addded x_new coordinates (%d) (%f, %f) \n", i, added_x_new->state.loc.x(), added_x_new->state.loc.y());
      visualization::DrawPoint(added_x_new->state.loc, 0xFF0000, global_viz_msg_);
      viz_pub_.publish(global_viz_msg_);
      node_ptrs_.push_back(added_x_new);
      for(auto other_near: x_near)
      {
        //printf("Other near %p\n", other_near);
        if (other_near == root_)
          continue;
        //printf("Looking at other near %d, (%f, %f) \n", i, other_near->state.loc.x(), other_near->state.loc.y());
        double c_near = other_near->cost;
        // Check kinematic feasibility of going from other_near to x_new
        State other_x_new = Steer(added_x_new->state, other_near->state.loc, &curvature, &distance);
        double c_new = added_x_new->cost + fabs(distance);
        // check if new cost is smaller and the closest steering input is actuall close to the desired x_new      
        if (c_new < c_near && (other_x_new.loc - other_near->state.loc).norm() <= 1e-4)
        {
          if (CollisionFree(other_near->state, curvature, distance, map_cloud_))
          {
            //printf("Updating other near\n");
            TreeNode* old_parent = other_near->parent;
            //printf("Other near parent %p\n", old_parent);
            old_parent->RemoveChild(other_near);
            other_near->parent = added_x_new;
            added_x_new->AddExistingChild(other_near, std::make_pair(curvature, distance));
            other_near->cost = c_new;
            // printf("Finished updating other near\n");
          }
        }
      }
      if ((added_x_new->state.loc-x_goal_).norm() <= GOAL_RADIUS)
      {
        goalNodes[added_x_new] = added_x_new->cost;
      }
    }
  }
  // Create waypoints with the neccessarry curvatures to reach them
  std::vector<std::pair<double, Vector2f>> output;
  while (x_best != nullptr && x_best->parent != nullptr)
  {
    if(!output.empty() && output.back().first == x_best->parent->children[x_best].first) {
      output.back().second = x_best->state.loc;
    }
    else {
      output.push_back(std::make_pair(x_best->parent->children[x_best].first, x_best->state.loc));
    }
    x_best = x_best->parent;
  }

  return output;
}

std::vector<std::pair<double, Vector2f>> RRT::KinodynamicRRT(std::vector<Eigen::Vector2f>& points, int max_iterations, double costGap, double optimalCost, double improvement_iterations)
{
  visualization::ClearVisualizationMsg(global_viz_msg_);
  global_viz_msg_ = map_viz_msg_;
  cout << "Planning with Kinodynamic RRT\n" << std::endl;
  // Convert pointcloud to Map frame
  getMapPointCloud(points);
  visualization::DrawCross(x_start_, 0.3, 0x0000FF, global_viz_msg_);
  visualization::DrawCross(x_goal_, 0.3, 0x0000FF, global_viz_msg_);

  std::map<TreeNode*, double> goalNodes;
  TreeNode* x_best = nullptr;
  double c_best = std::numeric_limits<double>::infinity();
  bool found_init_solution = false;
  int num_improve_iter = 0;
  for (int i = 0; i < max_iterations; ++i)
  {
    if (costGap > 0 && (c_best-optimalCost)/optimalCost < costGap)
      break;
    for(auto gn: goalNodes)
    {
      if (gn.second < c_best)
      {
        printf("new c_best (%f)\n", gn.second);
        found_init_solution = true;
        c_best = gn.second;
        x_best = gn.first;
        c_best_overall = c_best;
      }
    }

    if (found_init_solution)
      num_improve_iter++;
    if (improvement_iterations > 0 && improvement_iterations < num_improve_iter)
      break;

    Vector2f x_rand = Sample(std::numeric_limits<double>::infinity());
    TreeNode* x_nearest = Nearest(x_rand);
    if (x_nearest == nullptr)
    {
      printf("xnearest is null\n");
      continue;
    }
    double curvature;
    double distance;
    State x_new = Steer(x_nearest->state, x_rand, &curvature, &distance);
    if (CollisionFree(x_nearest->state, curvature, distance, map_cloud_))
    {
      std::vector<TreeNode*> x_near = Near(x_new, 1.0f);
      TreeNode* x_min = x_nearest;
      double c_min = x_nearest->cost + fabs(distance);
      double curvature_min = curvature;
      double distance_min = distance;
      for(auto other_near: x_near)
      {
        //printf("Other near %p\n", other_near);
        // Check kinematic feasibility of going from other_near to x_new
        State other_x_new = Steer(other_near->state, x_new.loc, &curvature, &distance);
        double c_new = other_near->cost + fabs(distance);
        // check if new cost is smaller and the closest steering input is actuall close to the desired x_new
        if (c_new < c_min && (other_x_new.loc - x_new.loc).norm() <= 1e-4)
        {
          if (CollisionFree(other_near->state, curvature, distance, map_cloud_))
          {
            x_min = other_near;
            c_min = c_new;
            curvature_min = curvature;
            distance_min = distance;
          }
        }
      }
      TreeNode* added_x_new = x_min->AddNewChild(x_new, c_min, std::make_pair(curvature_min, distance_min));
      //printf("Addded x_new coordinates (%d) (%f, %f) \n", i, added_x_new->state.loc.x(), added_x_new->state.loc.y());
      visualization::DrawPoint(added_x_new->state.loc, 0xFF0000, global_viz_msg_);
      viz_pub_.publish(global_viz_msg_);
      node_ptrs_.push_back(added_x_new);
      for(auto other_near: x_near)
      {
        //printf("Other near %p\n", other_near);
        if (other_near == root_)
          continue;
        //printf("Looking at other near %d, (%f, %f) \n", i, other_near->state.loc.x(), other_near->state.loc.y());
        double c_near = other_near->cost;
        // Check kinematic feasibility of going from other_near to x_new
        State other_x_new = Steer(added_x_new->state, other_near->state.loc, &curvature, &distance);
        double c_new = added_x_new->cost + fabs(distance);
        // check if new cost is smaller and the closest steering input is actuall close to the desired x_new      
        if (c_new < c_near && (other_x_new.loc - other_near->state.loc).norm() <= 1e-4)
        {
          if (CollisionFree(other_near->state, curvature, distance, map_cloud_))
          {
            //printf("Updating other near\n");
            TreeNode* old_parent = other_near->parent;
            //printf("Other near parent %p\n", old_parent);
            old_parent->RemoveChild(other_near);
            other_near->parent = added_x_new;
            added_x_new->AddExistingChild(other_near, std::make_pair(curvature, distance));
            other_near->cost = c_new;
            // printf("Finished updating other near\n");
          }
        }
      }
      if ((added_x_new->state.loc-x_goal_).norm() <= GOAL_RADIUS)
      {
        goalNodes[added_x_new] = added_x_new->cost;
      }
    }
  }
  // Create waypoints with the neccessarry curvatures to reach them
  std::vector<std::pair<double, Vector2f>> output;
  while (x_best != nullptr && x_best->parent != nullptr)
  {
    if(!output.empty() && output.back().first == x_best->parent->children[x_best].first) {
      output.back().second = x_best->state.loc;
    }
    else {
      output.push_back(std::make_pair(x_best->parent->children[x_best].first, x_best->state.loc));
    }
    x_best = x_best->parent;
  }

  return output;
}

std::vector<Vector2f> RRT::LinearInformedRRT(std::vector<Eigen::Vector2f>& points, int max_iterations, double costGap, double optimalCost, double improvement_iterations)
{
  visualization::ClearVisualizationMsg(global_viz_msg_);
  global_viz_msg_ = map_viz_msg_;
  cout << "Planning with Linear Informed RRT\n" << std::endl;
  c_best_overall = 0;
  // Convert pointcloud to Map frame
  getMapPointCloud(points);
  visualization::DrawCross(x_start_, 0.3, 0x0000FF, global_viz_msg_);
  visualization::DrawCross(x_goal_, 0.3, 0x0000FF, global_viz_msg_);

  std::map<TreeNode*, double> goalNodes;
  TreeNode* x_best = nullptr;
  double c_best = std::numeric_limits<double>::infinity();
  bool found_init_solution = false;
  int num_improve_iter = 0;
  for (int i = 0; i < max_iterations; ++i)
  {
    if (costGap > 0 && (c_best-optimalCost)/optimalCost < costGap)
      break;
    for(auto gn: goalNodes)
    {
      if (gn.second < c_best)
      {
        printf("new c_best (%f)\n", gn.second);
        found_init_solution = true;
        c_best = gn.second;
        x_best = gn.first;
        c_best_overall = c_best;
      }
    }

    if (found_init_solution) {
      num_improve_iter++;
    }
    if (improvement_iterations > 0 && improvement_iterations < num_improve_iter)
      break;

    Vector2f x_rand = Sample(c_best);
    while (x_rand.x() > x_bounds_.second || x_rand.x() < x_bounds_.first || x_rand.y() > y_bounds_.second || x_rand.y() < y_bounds_.first ) {
      // printf("X rand (%f, %f) is out of boundsd \n", x_rand.x(), x_rand.y());
      x_rand = Sample(c_best);
    }
    TreeNode* x_nearest = Nearest(x_rand);
    if (x_nearest == nullptr)
    {
      printf("xnearest is null\n");
      continue;
    }
    State x_new = SteerLinear(x_nearest->state, x_rand);
    if (CollisionFreeLinear(x_nearest->state, x_new, map_cloud_))
    {
      std::vector<TreeNode*> x_near = Near(x_new, 1.0f);
      TreeNode* x_min = x_nearest;
      double distance_min = (x_nearest->state.loc - x_new.loc).norm();
      double c_min = x_nearest->cost + (x_nearest->state.loc - x_new.loc).norm();
      for(auto other_near: x_near)
      {
        double c_new = other_near->cost + (other_near->state.loc - x_new.loc).norm();
        // check if new cost is smaller and the closest steering input is actuall close to the desired x_new
        if (c_new < c_min)
        {
          if (CollisionFreeLinear(other_near->state, x_new, map_cloud_))
          {
            x_min = other_near;
            c_min = c_new;
            distance_min = (other_near->state.loc - x_new.loc).norm();
          }
        }
      }
      TreeNode* added_x_new = x_min->AddNewChild(x_new, c_min, std::make_pair(distance_min, distance_min));
      //printf("Addded x_new coordinates (%d) (%f, %f) \n", i, added_x_new->state.loc.x(), added_x_new->state.loc.y());
      visualization::DrawPoint(added_x_new->state.loc, 0xFF0000, global_viz_msg_);
      viz_pub_.publish(global_viz_msg_);
      node_ptrs_.push_back(added_x_new);
      for(auto other_near: x_near)
      {
        if (other_near == root_)
          continue;
        double c_near = other_near->cost;
        double distance = (added_x_new->state.loc - other_near->state.loc).norm();
        double c_new = added_x_new->cost + fabs(distance);
        // check if new cost is smaller and the closest steering input is actuall close to the desired x_new      
        if (c_new < c_near)
        {
          if (CollisionFreeLinear(other_near->state, added_x_new->state, map_cloud_))
          {
            TreeNode* old_parent = other_near->parent;
            old_parent->RemoveChild(other_near);
            other_near->parent = added_x_new;
            added_x_new->AddExistingChild(other_near, std::make_pair(distance, distance));
            other_near->cost = c_new;
          }
        }
      }
      if ((added_x_new->state.loc-x_goal_).norm() <= GOAL_RADIUS)
      {
        goalNodes[added_x_new] = added_x_new->cost;
      }
    }
  }
  // Create waypoints with the neccessarry curvatures to reach them
  std::vector<Vector2f> output;
  while (x_best != nullptr && x_best->parent != nullptr)
  {
    output.push_back(x_best->state.loc);
    x_best = x_best->parent;
  }
  return output;
}

std::vector<Vector2f> RRT::LinearRRT(std::vector<Eigen::Vector2f>& points, int max_iterations, double costGap, double optimalCost, double improvement_iterations)
{
  visualization::ClearVisualizationMsg(global_viz_msg_);
  global_viz_msg_ = map_viz_msg_;
  cout << "Planning with Linear RRT\n" << std::endl;
  c_best_overall = 0;
  // Convert pointcloud to Map frame
  getMapPointCloud(points);
  visualization::DrawCross(x_start_, 0.3, 0x0000FF, global_viz_msg_);
  visualization::DrawCross(x_goal_, 0.3, 0x0000FF, global_viz_msg_);

  std::map<TreeNode*, double> goalNodes;
  TreeNode* x_best = nullptr;
  double c_best = std::numeric_limits<double>::infinity();
  bool found_init_solution = false;
  int num_improve_iter = 0;
  for (int i = 0; i < max_iterations; ++i)
  {
    if (costGap > 0 && (c_best-optimalCost)/optimalCost < costGap)
      break;
    for(auto gn: goalNodes)
    {
      if (gn.second < c_best)
      {
        printf("new c_best (%f)\n", gn.second);
        found_init_solution = true;
        c_best = gn.second;
        x_best = gn.first;
        c_best_overall = c_best;
      }
    }
    if (found_init_solution)
      num_improve_iter++;
    if (improvement_iterations > 0 && improvement_iterations < num_improve_iter)
      break;
    // Main difference is how we sample. In regular rrt* we just have uniform sampling
    Vector2f x_rand = Sample(std::numeric_limits<double>::infinity());
    TreeNode* x_nearest = Nearest(x_rand);
    if (x_nearest == nullptr)
    {
      printf("xnearest is null\n");
      continue;
    }
    State x_new = SteerLinear(x_nearest->state, x_rand);
    if (CollisionFreeLinear(x_nearest->state, x_new, map_cloud_))
    {
      std::vector<TreeNode*> x_near = Near(x_new, 1.0f);
      TreeNode* x_min = x_nearest;
      double distance_min = (x_nearest->state.loc - x_new.loc).norm();
      double c_min = x_nearest->cost + (x_nearest->state.loc - x_new.loc).norm();
      for(auto other_near: x_near)
      {
        double c_new = other_near->cost + (other_near->state.loc - x_new.loc).norm();
        // check if new cost is smaller and the closest steering input is actuall close to the desired x_new
        if (c_new < c_min)
        {
          if (CollisionFreeLinear(other_near->state, x_new, map_cloud_))
          {
            x_min = other_near;
            c_min = c_new;
            distance_min = (other_near->state.loc - x_new.loc).norm();
          }
        }
      }
      TreeNode* added_x_new = x_min->AddNewChild(x_new, c_min, std::make_pair(distance_min, distance_min));
      //printf("Addded x_new coordinates (%d) (%f, %f) \n", i, added_x_new->state.loc.x(), added_x_new->state.loc.y());
      visualization::DrawPoint(added_x_new->state.loc, 0xFF0000, global_viz_msg_);
      viz_pub_.publish(global_viz_msg_);
      node_ptrs_.push_back(added_x_new);
      for(auto other_near: x_near)
      {
        if (other_near == root_)
          continue;
        double c_near = other_near->cost;
        double distance = (added_x_new->state.loc - other_near->state.loc).norm();
        double c_new = added_x_new->cost + fabs(distance);
        // check if new cost is smaller and the closest steering input is actuall close to the desired x_new      
        if (c_new < c_near)
        {
          if (CollisionFreeLinear(other_near->state, added_x_new->state, map_cloud_))
          {
            TreeNode* old_parent = other_near->parent;
            old_parent->RemoveChild(other_near);
            other_near->parent = added_x_new;
            added_x_new->AddExistingChild(other_near, std::make_pair(distance, distance));
            other_near->cost = c_new;
          }
        }
      }
      if ((added_x_new->state.loc-x_goal_).norm() <= GOAL_RADIUS)
      {
        goalNodes[added_x_new] = added_x_new->cost;
      }
    }
  }
  // Create waypoints with the neccessarry curvatures to reach them
  std::vector<Vector2f> output;
  while (x_best != nullptr && x_best->parent != nullptr)
  {
    output.push_back(x_best->state.loc);
    x_best = x_best->parent;
  }
  return output;
}


void RRT::getMapPointCloud(const std::vector<Eigen::Vector2f>& points) {
  map_cloud_.reserve(points.size());

  const Rotation2Df rot_matrix(x_start_heading_);

  for (const auto& point:points) {
    map_cloud_.push_back(rot_matrix*point + x_start_);
  }
}

}  // namespace rrt
