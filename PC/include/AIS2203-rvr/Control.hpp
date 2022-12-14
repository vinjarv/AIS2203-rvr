#pragma once

#include <thread>
#include <atomic>

#include "ImageStream.hpp"
#include "RVRController.hpp"
#include "RVRData.hpp"
#include "SerialJoystick.hpp"
#include "PointMap.hpp"

class Control{
public:
    Control(const std::string& ip, const std::string& map_file);
    ~Control();

    void setManualMode(bool state);
    float manual_speed = 1;

    SerialJoystick joystick;
    ImageStream imagestream;
    RVRController controller;
    RVRData rvrdata;
    PointMap map;
private:
    void background_task();
    void loop();
    bool isNear(const std::string& id, float tolerance);

    int state = 0;
    std::atomic<bool> manual_enabled;
    std::atomic<bool> stop_requested;
    std::thread background_thread;
};
