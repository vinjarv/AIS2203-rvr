#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <map>
#include <string>
#include <optional>
#include <fstream>
#include <iostream>

#include "nlohmann/json.hpp"

// Represents a node on the map
struct NavPoint{
    // Constructors
    NavPoint() = default;
    NavPoint(float x, float y, std::string id) :
        x(x),
        y(y),
        id(std::move(id)) {};
    NavPoint(float x, float y, std::string id, std::vector<std::string> adj) :
        x(x),
        y(y),
        id(std::move(id)),
        adj(std::move(adj)) {};

    float x = 0.0f;
    float y = 0.0f;
    std::string id;
    std::vector<std::string> adj = {};
};

class PointMap{
public:
    explicit PointMap(const std::string& filename)
    {
        try {
            std::fstream fs {filename};
            const auto j = nlohmann::json::parse(fs);
            const auto& nodes = j["nodes"];
            // Iterate over nodes and add to list
            for (const auto& node : nodes.items()) {
                NavPoint np (
                    node.value()["x"],
                    node.value()["y"],
                    std::string(node.key())
                );
                // Iterate over string and get adjacent node ids
                // Trim spaces, separate at comma
                const std::string connected = node.value()["connected_nodes"];
                std::string s{};
                for (const char& c : connected){
                    if(c == ' ') {
                        continue;
                    } else if (c == ',') {
                        np.adj.emplace_back(s);
                        s = {};
                    } else {
                        s += c;
                    }
                }
                if (!s.empty()) {
                    np.adj.emplace_back(s);
                }
                // Store nav point
                points.emplace(np.id, np);
            }

        } catch (std::exception& ex) {
            std::cout << "[PointMap]: Error reading file \"" << filename << "\"\n" << ex.what() << std::endl;
        }
    }

    // A* search for path from start to end
    // Implemented based on https://en.wikipedia.org/wiki/A*_search_algorithm
    [[nodiscard]] std::optional<std::vector<NavPoint>> search(const std::string id_start, const std::string id_end) const
    {
        if (points.empty())
            return std::nullopt;

        try
        {
            std::map<std::string, NavPoint> opened_nodes;
            opened_nodes.emplace(id_start, points.at(id_start));
            std::map<std::string, std::string> prev_nodes {};

            // Construct map of scores, initialize with infinity
            std::map<std::string, float> g_score; // Cheapest path to node from start
            std::map<std::string, float> f_score; // Guess for cost of path through this node
            for (const auto& p : points) {
                g_score.emplace( p.first, std::numeric_limits<float>::infinity() );
                f_score.emplace( p.first, std::numeric_limits<float>::infinity() );
            }
            g_score.at(id_start) = 0;
            f_score.at(id_start) = nodeDistance(points.at(id_start), points.at(id_end));

            while (!opened_nodes.empty())
            {
                // Move to node in opened nodes with lowest f_score
                NavPoint current_node;
                float best = std::numeric_limits<float>::infinity();
                for (const auto& opened_node : opened_nodes) {
                    if (f_score.at(opened_node.first) < best){
                        best = f_score.at(opened_node.first);
                        current_node = opened_node.second;
                    }
                }
                // Check if goal reached
                if (current_node.id == id_end) {
                    return reconstructPath(prev_nodes, current_node);
                }
                // Remove current node from open nodes
                opened_nodes.erase(current_node.id);
                // Search neighbours
                for (const std::string& neighbour : current_node.adj) {
                    // Tentative score is cost of path to neighbour through current - it might not be the best path to that node!
                    float tentative_g_score = g_score.at(current_node.id)
                                              + nodeDistance(current_node, points.at(neighbour));
                    if (tentative_g_score < g_score.at(neighbour)) {
                        // This is the best path, save it
                        prev_nodes[neighbour] = current_node.id;
                        g_score[neighbour] = tentative_g_score;
                        f_score[neighbour] = tentative_g_score + nodeDistance(points.at(neighbour), points.at(id_end));
                        // Add to opened nodes
                        if (opened_nodes.count(neighbour) <= 0) {
                            opened_nodes.emplace(neighbour, points.at(neighbour));
                        }
                    }
                }
            }

            // If opened set is now empty, no path was found
            return std::nullopt;

        } catch (std::exception& ex)
        {
            std::cout << "[PointMap] Search error: " << ex.what() << std::endl;
            return std::nullopt;
        }
    }

    std::map<std::string, NavPoint> points;

private:

    // A* heuristic function
    [[nodiscard]] float nodeDistance(const NavPoint& a, const NavPoint& b) const
    {
        return std::hypot(b.x - a.x, b.y - a.y);
    }

    // A* path reconstruction function
    [[nodiscard]] std::vector<NavPoint> reconstructPath(const std::map<std::string, std::string>& prev_nodes, NavPoint current_node) const
    {
        // prev_node contains pairs of (node, prev_node) that links a node to its best previous node
        // Traversing this structure gives the best path backwards from the goal to the start
        // Begin at the end
        std::vector<NavPoint> total_path {current_node};
        // Termination criteria: current node not in list of visited nodes - this must be the start
        while ( prev_nodes.count(current_node.id) > 0 ) {
            // Move to previous node from current
            current_node = points.at(prev_nodes.at(current_node.id));
            // Prepend to list
            total_path.insert(total_path.begin(), current_node);
        }
        return total_path;
    }
};
