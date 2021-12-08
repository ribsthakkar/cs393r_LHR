#include <vector>
#include <map>
#include <utility>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"

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

struct State {
  State();
  State(const Eigen::Vector2f& loc, double heading);
  Eigen::Vector2f loc;
  double heading;
};

struct TreeNode
{
  TreeNode();
  TreeNode(const Eigen::Vector2f& loc, double heading, TreeNode* parent=NULL);
  TreeNode* parent;
  State state;
  std::map<TreeNode*, std::pair<double, double>> children;
  double cost; 

  TreeNode* AddNewChild(State& child, double cost, std::pair<double, double> control);
  void AddExistingChild(TreeNode*, std::pair<double, double> control);
  void RemoveChild(TreeNode*);
};

class RRT {
 public:
  // RRT();
  RRT(Eigen::Vector2f x_start, double x_start_heading, Eigen::Vector2f x_goal, double x_goal_heading, std::pair<double, double> x_bounds_, std::pair<double, double> y_bounds_, const vector_map::VectorMap& map);

  ~RRT();
  Eigen::Vector2f Sample(double c_max);

  // Given a randdom point, find the  nearest node that exists in the tree
  TreeNode* Nearest(Eigen::Vector2f& x_rand);

  // Given the nearest node on the tree, and the sampled point, this goes through a bunch
  // of steering options to find which one brings us the closest to the goal.
  // It returns the state (tree node) resulting from that move, and sets the curvature and travel of the move
  State Steer(State& x_nearest, Eigen::Vector2f& x_rand, double* curvature, double* travel);

  // Given the point that can be steered to, returns list of points within some radius
  std::vector<TreeNode*> Near(State& x_new, double neighborhood_radius);

  bool CollisionFree(State& x_nearest, double curvature, double distance, std::vector<Eigen::Vector2f> obsevation_points);

  // Run the informed RRT* algorithm
  std::vector<std::pair<double, Eigen::Vector2f>> InformedRRT(std::vector<Eigen::Vector2f>& points, int max_iterations=10000);

  // Sets the map_cloud_ variable to represent the pointcloud in the map frame
  void getMapPointCloud(const std::vector<Eigen::Vector2f>& points);

 private:
  // Initial State
  Eigen::Vector2f x_start_;
  double x_start_heading_;
  // Goal State
  Eigen::Vector2f x_goal_;
  double x_goal_heading_;

  // Random number generator
  util_random::Random rng_;

  // Bounds on state space
  std::pair<double, double> x_bounds_;
  std::pair<double, double> y_bounds_;

  // World Map
  vector_map::VectorMap map_;

  // Save the problem ellipse here
  Ellipse ellipse_;

  // Possible steering angles
  std::vector<double> turning_radii_;

  // Tree Root
  TreeNode root_;

  //TreeNode pointers to store and clean up preventing memory leaks
  std::vector<TreeNode*> node_ptrs_;

  // Point cloud in map frame
  std::vector<Eigen::Vector2f> map_cloud_;
};

} // namespace rrt
