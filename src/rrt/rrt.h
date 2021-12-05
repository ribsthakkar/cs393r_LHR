#include <vector>
#include <utility>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/util/random.h"

namespace rrt {

struct Ellipse {
  Eigen::Vector2f start;
  Eigen::Vector2f goal;
  Eigen::Vector2f centre;
  Eigen::Rotation2Df rotation;
  double c_min;

  Ellipse(){};
  Ellipse(const Eigen::Vector2f& start_point, const Eigen::Vector2f& goal_point);
};

class RRT {
 public:
  Eigen::Vector2f Sample(double c_max);
  Eigen::Vector2f Nearest(Eigen::Vector2f& x_rand);
  Eigen::Vector2f Steer(Eigen::Vector2f& x_nearest, Eigen::Vector2f& x_rand);
  Eigen::Vector2f Near(Eigen::Vector2f& x_near);

 private:
  // Random number generator
  util_random::Random rng_;

  // Bounds on state space
  std::pair<double, double> x_bounds_;
  std::pair<double, double> y_bounds_;

  // Save the problem ellipse here
  Ellipse ellipse_;
};

} // namespace rrt
