#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <math.h> 
#include <limits>

#include "eigen3/Eigen/Dense"
#include "math/line2d.h"
#include "vector_map/vector_map.h"
#include "graph/graph.h"
#include "shared/math/geometry.h"
#include "navigation/simple_queue.h"

namespace graph {

    Graph::Graph(const vector_map::VectorMap& map, float precision):
     precision(precision){
        map_ = map;
        std::string prefix = map.file_name.substr(0, map.file_name.length()-4);
        std::string pre_calculated_file = prefix + ".graph";
        FILE* fid = fopen(pre_calculated_file.c_str(), "r");
        if (fid == NULL) {
            // Precomputed graph file doesn't exist. Let's create it
            fid = fopen(pre_calculated_file.c_str(), "rw");
            float minx(100000.0f), miny(100000.0f), maxx(-100000.0f), maxy(100000.0f);
            for (size_t j = 0; j < map_.lines.size(); ++j) {
                const geometry::line2f map_line = map_.lines[j];
                minx = std::min(minx, std::min(map_line.p0.x(), map_line.p1.x()));
                miny = std::min(minx, std::min(map_line.p0.y(), map_line.p1.y()));
                maxx = std::max(maxx, std::max(map_line.p0.x(), map_line.p1.x()));
                maxy = std::max(maxy, std::max(map_line.p0.y(), map_line.p1.y()));
            }
            minx = trunc(minx-1);
            miny = trunc(miny-1);
            maxx = trunc(maxx+1);
            maxy = trunc(maxy+1);
            for(float x1 = minx; x1 <= maxx; x1 += precision) {
                for (float y1 = miny; y1 <=maxy; y1 += precision) {
                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            if (dx == 0 && dy == 0) continue;
                            float x2 = x1 + dx * precision;
                            float y2 = y1 + dy * precision;
                            if (!isEdgeValid(Eigen::Vector2f(x1, y1), Eigen::Vector2f(x2, y2))) continue;
                            std::pair<int, int> src = std::make_pair(round(x1*100), round(y1*100));
                            std::pair<int, int> dest = std::make_pair(round(x2*100), round(y2*100));
                            if (nodes.find(src) == nodes.end()) {
                                nodes[src] = Node(src);
                            }
                            if (nodes.find(dest) == nodes.end()) {
                                nodes[dest] = Node(src);
                            }
                            float cost = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
                            nodes[src].add_adjacent(nodes.at(dest), cost);
                            edges.push_back(Edge(nodes.at(src), nodes.at(dest), cost));
                            // write edge to file
                            fprintf(fid, "%f,%f,%f,%f", x1, y1, x2, y2);
                        }
                    }
                }
            }
        }
        else
        {
            // Parse the precomputed graph file
            float x1(0), y1(0), x2(0), y2(0);
            while (fscanf(fid, "%f,%f,%f,%f", &x1, &y1, &x2, &y2) == 4) {
                std::pair<int, int> src = std::make_pair(round(x1*100), round(y1*100));
                std::pair<int, int> dest = std::make_pair(round(x2*100), round(y2*100));
                if (nodes.find(src) == nodes.end()) {
                    nodes[src] = Node(src);
                }
                if (nodes.find(dest) == nodes.end()) {
                    nodes[dest] = Node(src);
                }
                float cost = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
                nodes[src].add_adjacent(nodes.at(dest), cost);
                edges.push_back(Edge(nodes.at(src), nodes.at(dest), cost));
            }
        }
    }
    bool Graph::isEdgeValid(Eigen::Vector2f edge_p1, Eigen::Vector2f edge_p2)
    {
        for (size_t j = 0; j < map_.lines.size(); ++j) {
            const geometry::line2f map_line = map_.lines[j];
            // Need to make this configurable
            if (geometry::MinDistanceLineLine(edge_p1, edge_p2, map_line.p0, map_line.p1) <= 0.1) 
                return false;
        }
        return true;
    }

    std::vector<Eigen::Vector2f> Graph::ShortestPath(Eigen::Vector2f robot_loc, Eigen::Vector2f target_loc) {
        std::pair<int, int> nearest_src = std::make_pair(round(fmod(robot_loc.x(), precision) * 100), round(fmod(robot_loc.y(), precision) * 100));
        std::pair<int, int> nearest_dest = std::make_pair(round(fmod(target_loc.x(), precision) * 100), round(fmod(target_loc.y(), precision) * 100));
        if (nodes.find(nearest_src) == nodes.end() || nodes.find(nearest_dest) == nodes.end())
            return std::vector<Eigen::Vector2f>();
        std::map<Node, float> weights;
        for(auto& n_pair: nodes)
        {
            weights[n_pair.second] = std::numeric_limits<float>::infinity();
        }
        Node& src = nodes[nearest_src];
        Node& dest = nodes[nearest_dest];
        SimpleQueue<Node, float> q;
        std::map<Node, Node> parent;
        parent[src] = src;
        weights[src] = 0.0f;
        q.Push(src, 0.0f);
        while (!q.Empty()) {
            const Node& current = q.Pop();
            if (current == dest)
                break;
            for (auto& p: current.getAdjacent())
            {
                auto& n = p.first;
                float new_weight = weights[current] + p.second;
                if (new_weight < n.getWeight()) {
                    weights[n] = new_weight;
                    q.Push(n, new_weight + Heuristic(current, dest));
                    parent[n] = current;
                }
            }
        }
        std::vector<Eigen::Vector2f> path;
        Node& curr_p = dest;
        while (!(curr_p == src)) {
            path.push_back(Eigen::Vector2f(curr_p.id.first/100.0f, curr_p.id.second/100.0f));
            curr_p = parent[curr_p];
        }
        path.push_back(Eigen::Vector2f(src.id.first/100.0f, src.id.second/100.0f));
        return path;
    }
    float Graph::Heuristic(const Node& curr, const Node& dest) {
        return fabs((curr.id.first - dest.id.first)/100.0f) + fabs((curr.id.second - dest.id.second)/100.0f);
    }

    Node::Node(): id(0, 0), weight(std::numeric_limits<float>::infinity()) {}
    Node::Node(std::pair<int, int> id): id(id), weight(std::numeric_limits<float>::infinity()) {}
    void Node::add_adjacent(const Node& other, float other_cost) {
        adjacent[other] = other_cost;
    }
    float Node::getWeight() const {
        return weight;
    }
    const std::map<Node, float>& Node::getAdjacent() const {
        return adjacent;
    }
    void Node::setWeight(float new_weight) {
        weight = new_weight;
    }


    Edge::Edge(const Node& src, const Node& dest, double cost): src(src), dest(dest), cost(cost) {}
    double Edge::getCost() {
        return cost;
    }
    const Node& Edge::getSrc() const {
        return src;
    }
    const Node& Edge::getDest() const {
        return dest;
    }

}  // namespace graph
