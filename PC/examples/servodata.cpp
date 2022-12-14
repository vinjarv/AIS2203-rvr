#include <iostream>
#include <string>
#include <nlohmann/json.hpp>
#include "zmqpp/zmqpp.hpp"

using json = nlohmann::json;

int main() {

    zmqpp::context_t context{};

    zmqpp::socket socket{context, zmqpp::socket_type::sub};
    socket.subscribe("");
    socket.connect("tcp://10.25.45.75:5555");

    zmqpp::message_t message;
    socket.receive(message);

    std::string json_str;

    message >> json_str;
    json sensor_data = json::parse(json_str);

    std::cout << sensor_data << std::endl;
    return 0;

}
