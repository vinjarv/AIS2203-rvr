#include <algorithm>
#include <string>
#include <fstream>
#include <iostream>

#include "nlohmann/json.hpp"
#include "point_map.hpp"

int main(int argc, char** argv)
{
    try{
        std::string filename {argv[1]};
        std::fstream fs {filename};
        const nlohmann::json j = nlohmann::json::parse(fs);
        const auto& nodes = j["nodes"];

        // ----- Parse manually -----
        std::cout << "Nav nodes:" << std::endl;
        for (const auto& node : nodes.items()) {
            if (node.value().contains("name"))
                std::cout << "Name: " << node.value()["name"] << "\t";
            else
                std::cout << "Name: None\t";

            std::cout << "\"" << node.key() << "\":" << node.value() << std::endl;
        }

        // ----- Parse with class -----
        std::cout << "\nParsing with PointMap class:" << std::endl;
        PointMap map {filename};
        // Print IDs and connected nodes
        for (const auto& p : map.points) {
            std::cout << "ID: " << p.second.id << " - Connected: ";
            for (const auto& a : p.second.adj) {
                std::cout << " " << a;
                if (a != p.second.adj.back())
                    std::cout << ",";
            }
            std::cout << std::endl;
        }

        // ----- Test search algorithm -----
        if (argc >= 4) {
            std::string start = argv[2];
            std::string end = argv[3];
            auto path = map.search(start, end);
            if (path.has_value()) {
                std::cout << "Path found: ";
                for (const auto &p: path.value()) {
                    std::cout << p.id << " -> ";
                }
                std::cout << "Done" << std::endl;
            } else {
                std::cout << "No path found" << std::endl;
            }
        }

        // ----- Find named nodes -----
        auto named = map.getNamedNodes();
        std::cout << "\nNamed nodes:" << std::endl;
        for (const auto& n : named){
            std::cout << n.id << " - " << n.name << std::endl;
        }

    } catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}
