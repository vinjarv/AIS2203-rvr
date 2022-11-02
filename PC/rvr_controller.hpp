#include "zmqpp/zmqpp.hpp"
#include <string>

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

    void driveManual(float speed_forward, float speed_rotation)
    {
        std::string command = {
            std::string{"{\"command\":\"manual\""} +
            std::string{"\"parameters\":{"} +
            std::string{"\"speed_forward\""} + std::to_string(speed_forward) + std::string{","} +
            std::string{"\"speed_rotation\":"} + std::to_string(speed_forward) +
            std::string{"}}"}
        };
        zmqpp::message_t msg;
        msg << command;
        socket.send(msg);
    }

    void driveAuto(float x, float y, float yaw)
    {
        std::string command =   {
            std::string{"{\"command\":\"auto\""} +
            std::string{"\"parameters\":{"} +
            std::string{"\"x\""} + std::to_string(x) + std::string{","} +
            std::string{"\"y\":"} + std::to_string(y) + "," +
            std::string{"\"yaw\":"} + std::to_string(yaw) +
            std::string{"}}"}
        };
        zmqpp::message_t msg;
        msg << command;
        socket.send(msg);
    }

    void driveStop() {
        this->driveManual(0, 0);
    }

    void setServo(int channel, float duty_cycle) {
        std::string command =   {
                std::string{"{\"command\":\"servo\""} +
                std::string{"\"parameters\":{"} +
                std::string{"\"channel\""} + std::to_string(channel) + std::string{","} +
                std::string{"\"duty_cycle\":"} + std::to_string(duty_cycle) +
                std::string{"}}"}
        };
        zmqpp::message_t msg;
        msg << command;
        socket.send(msg);
    }

private:
    zmqpp::context ctx;
    zmqpp::socket socket;
    zmqpp::endpoint_t endpoint;
};
