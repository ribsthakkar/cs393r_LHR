#include <algorithm>
#include <stack>
#include <limits>
#include "rrt.h"

#include "math.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/util/random.h"
#include "shared/math/math_util.h"
#include "vector_map/vector_map.h"

using Eigen::Vector2f;
using Eigen::Rotation2Df;

namespace rrt {

Vector2f sampleCircle(util_random::Random& rng, float radius=1.0) {
  const float theta = rng.UniformRandom(0, 2*M_PI);
  const float r = rng.UniformRandom(0, radius);

  return Vector2f(r*cos(theta), r*sin(theta));
}

Vector2f sampleEllipse(util_random::Random& rng, Ellipse& ellipse, double c_best) {
  if (math_util::definitelyGreaterThan(ellipse.c_min, c_best, 1e-7)) {
    throw std::invalid_argument("c_best can't be smaller than c_min!");
  }

  // See paper, construct L matrix from ellipse axes
  const auto L = (Vector2f(0.5*c_best, 0.5*sqrt(c_best*c_best - ellipse.c_min*ellipse.c_min))).asDiagonal();

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
  c_min = distance.norm();
  centre = start + 0.5*(distance);
  const float theta = atan2(distance.y(), distance.x());
  rotation = Rotation2Df(theta);
}

State::State(){}
State::State(const Eigen::Vector2f& loc, double heading):
  loc(loc), heading(heading) {}

TreeNode::TreeNode(): parent(nullptr), state(), children() {}
TreeNode::TreeNode(const Eigen::Vector2f& loc, double heading, TreeNode* parent):
  parent(parent),
  state(loc, heading),
  children() {}

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

// RRT::RRT() {}

RRT::RRT(Vector2f x_start_loc, double x_start_heading, Vector2f x_goal_loc, double x_goal_heading, std::pair<double, double> x_bounds, std::pair<double, double> y_bounds):
  x_start_(x_start_loc),
  x_start_heading_(x_start_heading),
  x_goal_(x_goal_loc),
  x_goal_heading_(x_goal_heading),
  x_bounds_(x_bounds),
  y_bounds_(y_bounds),
  ellipse_(x_start_loc, x_goal_loc),
  root_(x_start_loc, x_start_heading) {}

RRT::~RRT() {
  for (auto p: node_ptrs_)
  {
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
  TreeNode* currentNode = &root_;
  std::stack<TreeNode*> s;
  s.push(currentNode);
  while (!s.empty())
  {
    currentNode = s.top();
    s.pop();
    if ((currentNode->state.loc - x_rand).norm() < minDistance)
    {
      minDistance = (currentNode->state.loc - x_rand).norm();
      nearest = currentNode;
    }
    for (auto& c: currentNode->children)
    {
      s.push(c.first);
    }
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
  for (auto radius: turning_radii_) {

    // Check for r = infinity case
    if (fabs(1/radius) < 1e-5) {
      // Find line to goal
      const Vector2f nearest_to_goal = rot_matrix.inverse() * (x_rand - x_nearest.loc); // x_nearest frame

      // Clamp travel distance to goal
      double travel_distance = nearest_to_goal.x();
      math_util::Clamp(travel_distance, -1*max_dist, max_dist);

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
    const double max_alpha = fabs(max_dist / radius);

    // Calculate alpha to goal
    const Vector2f center_to_goal = x_rand - turn_center;
    const double goal_direction_sign = (rot_matrix * Vector2f(1,0)).dot(center_to_goal); 
    double alpha = copysign(acos((-1*radius_line.dot(center_to_goal))/(fabs(radius) * center_to_goal.norm())), goal_direction_sign); // Using dot product

    // Set alpha = min(max_alpha, alpha to goal)
    math_util::Clamp(alpha, -1*max_alpha, max_alpha);

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
      *curvature = 1/radius;
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
  TreeNode* currentNode = &root_;
  s.push(currentNode);
  while (!s.empty())
  {
    currentNode = s.top();
    s.pop();
    // Maybe we also needd to check if we can actually steer from this location to x_new
    if ((currentNode->state.loc - x_new.loc).norm() < neighborhood_radius)
    {
      neighborhood.push_back(currentNode);
    }
    for (auto& c: currentNode->children)
    {
      s.push(c.first);
    }
  }
  return neighborhood;
}

bool RRT::CollisionFree(State& x_nearest, double curvature, double distance, std::vector<Vector2f> local_observation_points)
{
  return true;
} 

std::vector<std::pair<double, Vector2f>> RRT::InformedRRT(std::vector<Eigen::Vector2f>& points, int max_iterations)
{
  std::map<TreeNode*, double> goalNodes;
  TreeNode* x_best = nullptr;
  double c_best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < max_iterations; ++i)
  {
    for(auto gn: goalNodes)
    {
      if (gn.second < c_best)
      {
        c_best = gn.second;
        x_best = gn.first;
      }
    }

    Vector2f x_rand = Sample(c_best);
    TreeNode* x_nearest = Nearest(x_rand);
    double curvature;
    double distance;
    State x_new = Steer(x_nearest->state, x_rand, &curvature, &distance);
    if (CollisionFree(x_nearest->state, curvature, distance, points))
    {
      std::vector<TreeNode*> x_near = Near(x_new, 5.0f);
      TreeNode* x_min = x_nearest;
      double c_min = x_nearest->cost + distance;
      double curvature_min = curvature;
      double distance_min = distance;
      for(auto other_near: x_near)
      {
        // Check kinematic feasibility of going from other_near to x_new
        State other_x_new = Steer(other_near->state, x_new.loc, &curvature, &distance);
        double c_new = other_near->cost + distance;
        // check if new cost is smaller and the closest steering input is actuall close to the desired x_new
        if (c_new < c_min && (other_x_new.loc - x_new.loc).norm() <= 1e-4)
        {
          if (CollisionFree(other_near->state, curvature, distance, points))
          {
            x_min = other_near;
            c_min = c_new;
            curvature_min = curvature;
            distance_min = distance;
          }
        }
      }
      TreeNode* added_x_new = x_min->AddNewChild(x_new, c_min, std::make_pair(curvature_min, distance_min));
      node_ptrs_.push_back(added_x_new);
      for(auto other_near: x_near)
      {
        double c_near = other_near->cost;
        // Check kinematic feasibility of going from other_near to x_new
        State other_x_new = Steer(other_near->state, added_x_new->state.loc, &curvature, &distance);
        double c_new = other_near->cost + distance;
        // check if new cost is smaller and the closest steering input is actuall close to the desired x_new
        if (c_new < c_near && (other_x_new.loc - added_x_new->state.loc).norm() <= 1e-4)
        {
          if (CollisionFree(other_near->state, curvature, distance, points))
          {
            TreeNode* old_parent = other_near->parent;
            old_parent->RemoveChild(other_near);
            other_near->parent = added_x_new;
            added_x_new->AddExistingChild(other_near, std::make_pair(curvature, distance));
          }
        }
      }
      if ((added_x_new->state.loc-x_goal_).norm() <= 0.5)
      {
        goalNodes[added_x_new] = (added_x_new->state.loc-x_goal_).norm();
      }
    }
  }

  // Create waypoints with the neccessarry curvatures to reach them
  std::vector<std::pair<double, Vector2f>> output;
  while (x_best->parent != nullptr)
  {
    output.push_back(std::make_pair(x_best->parent->children[x_best].first, x_best->state.loc));
    x_best = x_best->parent;
  }
  std::reverse(output.begin(), output.end());
  return output;
}


}  // namespace rrt
