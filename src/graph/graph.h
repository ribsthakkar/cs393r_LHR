#include <string>
#include <vector>
#include <map>

#include "eigen3/Eigen/Dense"
#include "math/line2d.h"
#include "vector_map/vector_map.h"

#ifndef GRAPH_H
#define GRAPH_H

namespace graph {

   struct Node {
        Node(void);
        explicit Node(std::pair<int, int> id);
        void add_adjacent(const Node& other, float other_cost);
        float getWeight() const;
        const std::map<Node, float>& getAdjacent() const;
        void update(float weight);
        void setWeight(float weight);

        std::pair<int, int> id;
        std::map<Node, float> adjacent;
        float weight; 

        bool operator< (const Node& b) const {
            return id < b.id;
        }

        bool operator == (const Node& b) const {
            return id == b.id;
        }
    };


    struct Edge {
        Edge(const Node& src, const Node& dest, double cost);
        double getCost();
        const Node& getSrc() const;
        const Node& getDest() const;

        const Node& src;
        const Node& dest;
        double cost;
    };

    struct Graph {
        Graph(const vector_map::VectorMap& map, float precision=10.0f);
        std::vector<Eigen::Vector2f> ShortestPath(Eigen::Vector2f robot_loc, Eigen::Vector2f target_loc); 
        bool isEdgeValid(Eigen::Vector2f edge_p0, Eigen::Vector2f edge_p1);
        float Heuristic(const Node& curr, const Node& dest);

        std::map<std::pair<int, int>, Node> nodes;
        std::vector<Edge> edges;
        float precision;
        vector_map::VectorMap map_;
    };

}  // namespace graph

#endif  // GRAPH_H