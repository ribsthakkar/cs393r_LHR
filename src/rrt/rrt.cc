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
  return Vector2f(0, 0);
}

Vector2f RRT::Steer(Eigen::Vector2f& x_nearest, Eigen::Vector2f& x_rand) {
  // TODO
  return Vector2f(0, 0);
}

Vector2f RRT::Near(Eigen::Vector2f& x_near) {
  // TODO
  return Vector2f(0, 0);
}

}  // namespace rrt
