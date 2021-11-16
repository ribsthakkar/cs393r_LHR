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
            printf("here\n");
            fid = fopen(pre_calculated_file.c_str(), "w");
            float minx(100000.0f), miny(100000.0f), maxx(-100000.0f), maxy(-100000.0f);
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
                            std::pair<int, int> src = std::make_pair(round(math_util::roundMultiple(x1, precision)*100), round(math_util::roundMultiple(y1, precision)*100));
                            std::pair<int, int> dest = std::make_pair(round(math_util::roundMultiple(x2, precision)*100), round(math_util::roundMultiple(y2, precision)*100));
                            if (!isEdgeValid(Eigen::Vector2f(x1, y1), Eigen::Vector2f(x2, y2))) continue;
                            if (nodes.find(src) == nodes.end()) {
                                nodes[src] = std::map<std::pair<int, int>, float>();
                            }
                            float cost = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
                            nodes[src][dest] = cost;
                            fprintf(fid, "%f,%f,%f,%f\n", x1, y1, x2, y2);
                        }
                    }
                }
            }
            fclose(fid);
        }
        else
        {
            // Parse the precomputed graph file
            float x1(0), y1(0), x2(0), y2(0);
            while (fscanf(fid, "%f,%f,%f,%f", &x1, &y1, &x2, &y2) == 4) {
                std::pair<int, int> src = std::make_pair(round(math_util::roundMultiple(x1, precision)*100), round(math_util::roundMultiple(y1, precision)*100));
                std::pair<int, int> dest = std::make_pair(round(math_util::roundMultiple(x2, precision)*100), round(math_util::roundMultiple(y2, precision)*100));
                if (nodes.find(src) == nodes.end()) {
                    nodes[src] = std::map<std::pair<int, int>, float>();
                }
                float cost = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
                nodes[src][dest] = cost;
            }
            fclose(fid);
        }
        printf("Finished setting up graph\n");
        for(auto& n:nodes)
        {
            printf("(%d, %d)\n", n.first.first, n.first.second);
        }
    }
    bool Graph::isEdgeValid(Eigen::Vector2f edge_p1, Eigen::Vector2f edge_p2)
    {
        for (size_t j = 0; j < map_.lines.size(); ++j) {
            const geometry::line2f map_line = map_.lines[j];
            // Need to make this configurable
            if (geometry::MinDistanceLineLine(edge_p1, edge_p2, map_line.p0, map_line.p1) <= 0.4) 
                return false;
        }
        return true;
    }

    std::vector<Eigen::Vector2f> Graph::ShortestPath(Eigen::Vector2f robot_loc, Eigen::Vector2f target_loc) {
        std::pair<int, int> src = std::make_pair(round(math_util::roundMultiple(robot_loc.x(), precision) * 100), round(math_util::roundMultiple(robot_loc.y(), precision) * 100));
        std::pair<int, int> dest = std::make_pair(round(math_util::roundMultiple(target_loc.x(), precision) * 100), round(math_util::roundMultiple(target_loc.y(), precision) * 100));
        printf("Robot Loc: (%d, %d)\n", src.first, src.second);
        printf("Target Loc: (%d, %d)\n", dest.first, dest.second);
        if (nodes.find(src) == nodes.end() || nodes.find(dest) == nodes.end())
            return std::vector<Eigen::Vector2f>();
        std::map<std::pair<int, int>, float> weights;
        for(auto& n_pair: nodes)
        {
            weights[n_pair.first] = std::numeric_limits<float>::infinity();
        }
        SimpleQueue<std::pair<int, int>, float> q;
        std::map<std::pair<int, int>, std::pair<int, int>> parent;
        parent[src] = src;
        weights[src] = 0.0f;
        q.Push(src, 0.0f);
        printf("Here\n");
        while (!q.Empty()) {
            const auto current = q.Pop();
            if (current == dest)
                break;
            for (auto p: nodes[current])
            {
                auto n = p.first;
                float new_weight = weights[current] + p.second;
                if (new_weight < weights[n]) {
                    weights[n] = new_weight;
                    q.Push(n, -(new_weight + Heuristic(current, dest)));
                    parent[n] = current;
                }
            }
        }
        std::vector<Eigen::Vector2f> path;
        auto& curr_p = dest;
        while (true) {
            path.push_back(Eigen::Vector2f(curr_p.first/100.0f, curr_p.second/100.0f));
            if (parent.find(curr_p) == parent.end())
            {
                path.clear();
                break;
            }
            if (parent.at(curr_p) == src) 
            {
                path.push_back(Eigen::Vector2f(src.first/100.0f, src.second/100.0f));
                break;
            }
            curr_p = parent[curr_p];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
    float Graph::Heuristic(const std::pair<int,int>& curr, const std::pair<int, int>& dest) const {
        return fabs((curr.first - dest.first)/100.0f) + fabs((curr.second - dest.second)/100.0f);
    }

}  // namespace graph
