#include <string>
#include <vector>
#include <map>

#include "eigen3/Eigen/Dense"
#include "math/line2d.h"
#include "vector_map/vector_map.h"

#ifndef GRAPH_H
#define GRAPH_H

namespace graph {
    struct Graph {
        Graph(const vector_map::VectorMap& map, float precision=0.5f);
        std::vector<Eigen::Vector2f> ShortestPath(Eigen::Vector2f robot_loc, Eigen::Vector2f target_loc); 
        bool isEdgeValid(Eigen::Vector2f edge_p0, Eigen::Vector2f edge_p1);
        float Heuristic(const std::pair<int,int>& curr, const std::pair<int, int>& dest) const;

        std::map<std::pair<int, int>, std::map<std::pair<int, int>, float>> nodes;
        float precision;
        vector_map::VectorMap map_;
    };

}  // namespace graph

#endif  // GRAPH_H