#include <stack>
#include "rrt.h"

#include "math.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/util/random.h"
#include "shared/math/math_util.h"

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

// RRT::RRT() {}

// RRT::RRT(Vector2f x_start_loc, x_start_heading, Vector2f x_goal_loc, Vector2f x_goal_heading, std::pair<double, double> x_bounds, std::pair<double, double> y_bounds):
//   x_start_(x_start),
//   x_start_heading_(x_start_heading),
//   x_goal_(x_goal),
//   x_goal_heading_(x_goal_heading),
//   x_bounds_(x_bounds_),
//   y_bounds_(y_bounds),
//   ellipse_(x_start_loc, x_goal_loc),
//   root_(x_start, x_start_heading) {}

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

Vector2f RRT::Nearest(Eigen::Vector2f& x_rand) {
  // TODO
  double minDistance = 100000000.0;
  Vector2f nearest;
  std::stack<TreeNode*> s;
  TreeNode* currentNode = &root_;
  s.push(currentNode);
  while (!s.empty())
  {
    currentNode = s.top();
    s.pop();
    if ((currentNode->state.loc - x_rand).norm() < minDistance)
    {
      minDistance = (currentNode->state.loc - x_rand).norm();
      nearest = currentNode->state.loc;
    }
    for (auto& c: currentNode->children)
    {
      s.push(c.second.get());
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

std::vector<TreeNode* > RRT::Near(TreeNode* x_new, double neighborhood_radius) {
  std::vector<TreeNode*> neighborhood;
  std::stack<TreeNode*> s;
  TreeNode* currentNode = &root_;
  s.push(currentNode);
  while (!s.empty())
  {
    currentNode = s.top();
    s.pop();
    // Maybe we also needd to check if we can actually steer from this location to x_new
    if ((currentNode->state.loc - x_new->state.loc).norm() < neighborhood_radius)
    {
      neighborhood.push_back(currentNode);
    }
    for (auto& c: currentNode->children)
    {
      s.push(c.second.get());
    }
  }
  return neighborhood;
}

}  // namespace rrt
