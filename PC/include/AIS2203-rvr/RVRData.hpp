#pragma once

#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "nlohmann/json.hpp"
#include "zmqpp/zmqpp.hpp"

class RVRData{
public:
    RVRData(const std::string& ip, const int port) : socket(context, zmqpp::socket_type::sub)
    {
        socket.subscribe("");
        socket.connect("tcp://" + ip + ":" + std::to_string(port));
        socket.set(zmqpp::socket_option::conflate, 1); // Buffer size 1

        background_thread = std::thread([&] {backgroundTask();});
    }

    ~RVRData()
    {
        stop_requested = true;
        background_thread.join();
    }

    struct RoverState {
        float x = 0.0f;
        float y = 0.0f;
        float yaw = 0.0f;
        float range_fw = 0.0f;
        float range_bw = 0.0f;
        std::vector<float> servo{0, 0, 0, 0};
    };

    RoverState getState(){
        std::lock_guard<std::mutex> lk(state_mutex);
        return state;
    }

private:
    void backgroundTask()
    {
        if (!stop_requested){
            loop();
        }
    }

    void loop()
    {
        using json = nlohmann::json;
        zmqpp::message_t message;
        socket.receive(message);
        std::string json_str;
        message >> json_str;
        json sensor_data = json::parse(json_str);

        std::lock_guard<std::mutex> lk(state_mutex);
        state.x = sensor_data["x"];
        state.y = sensor_data["y"];
        state.yaw = sensor_data["yaw"];
        state.range_fw = sensor_data["Range F"];
        state.range_bw = sensor_data["Range B"];
        state.servo[0] = sensor_data["Servo 1A"];
        state.servo[1] = sensor_data["Servo 1B"];
        state.servo[2] = sensor_data["Servo 2A"];
        state.servo[3] = sensor_data["Servo 2B"];
    }

    RoverState state;
    std::mutex state_mutex;
    zmqpp::context_t context;
    zmqpp::socket socket;
    std::thread background_thread;
    std::atomic<bool> stop_requested;
};
