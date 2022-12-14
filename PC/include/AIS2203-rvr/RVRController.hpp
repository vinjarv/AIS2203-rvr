#pragma once

#include "zmqpp/zmqpp.hpp"
#include <string>
#include <iostream>

class RVRController {
public:
    RVRController (std::string ip, unsigned int port) :
    ctx(zmqpp::context()),
    socket(zmqpp::socket(ctx, zmqpp::socket_type::pub)),
    endpoint("tcp://" + ip + ":" + std::to_string(port))
    {
        socket.connect(endpoint);
    }

    ~RVRController()
    {
        // Send "stop" command before disconnecting
        this->driveManual(0, 0);
    }

    void driveManual(const float speed_forward, const float speed_rotation)
    {
        std::string command = {
            std::string{"{\"command\":\"manual\","} +
            std::string{"\"parameters\":{"} +
            std::string{"\"speed_forward\":"} + std::to_string(speed_forward) + std::string{","} +
            std::string{"\"speed_rotation\":"} + std::to_string(speed_rotation) +
            std::string{"}}"}
        };
        sendCommand(command);
    }

    void driveAuto(const float x, const float y, const float yaw)
    {
        std::string command =   {
            std::string{"{\"command\":\"auto\","} +
            std::string{"\"parameters\":{"} +
            std::string{"\"x\":"} + std::to_string(x) + std::string{","} +
            std::string{"\"y\":"} + std::to_string(y) + "," +
            std::string{"\"yaw\":"} + std::to_string(yaw) +
            std::string{"}}"}
        };
        sendCommand(command);
    }

    void driveStop() {
        this->driveManual(0, 0);
    }

    void setServo(const int channel, const float angle_deg) {
        std::string command =   {
                std::string{"{\"command\":\"servo\","} +
                std::string{"\"parameters\":{"} +
                std::string{"\"channel\":"} + std::to_string(channel) + std::string{","} +
                std::string{"\"angle_deg\":"} + std::to_string(angle_deg) +
                std::string{"}}"}
        };
        sendCommand(command);
    }

    void setColour(const unsigned int r, const unsigned int g, const unsigned int b) {
        std::string command =   {
                std::string{"{\"command\":\"set_colour\","} +
                std::string{"\"parameters\":{"} +
                std::string{"\"r\":"} + std::to_string(r) + std::string{","} +
                std::string{"\"g\":"} + std::to_string(g) + std::string{","} +
                std::string{"\"b\":"} + std::to_string(b) +
                std::string{"}}"}
        };
        sendCommand(command);
    }

private:
    void sendCommand(const std::string& command){
        zmqpp::message_t msg;
        msg << command;
        socket.send(msg);
    }

    zmqpp::context ctx;
    zmqpp::socket socket;
    zmqpp::endpoint_t endpoint;
};
