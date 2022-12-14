#include "Control.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <thread>

Control::Control(const std::string& ip, const std::string& map_file)
    :   joystick(115200),
        rvrdata(ip, 5555),
        controller(ip, 8888),
        imagestream(ip, 8889),
        map(map_file)
{
    // Initialize joystick filter
    float q = 0.4f;
    float r = 4.0f;
    joystick.setKalmanQ_SD(q);
    joystick.setKalmanR_SD(r);

    // Start fetching camera data
    imagestream.init();
    imagestream.run();
    // Start running background_task() in a new thread
    background_thread = std::thread([&]{background_task();});
};

Control::~Control()
{
    stop_requested = true;
    background_thread.join();
}

void Control::background_task()
{
    while (!stop_requested) {
        loop();
    }
}

void Control::loop()
{
    if (manual_enabled) {
        state = 10000;
    }

    switch (state)
    {
        // Waiting for start signal
        case 0:
            if (false) {
                state = 1;
            }
            break;

        case 1:
            break;

        // Manual control
        case 10000:
            if (!manual_enabled) {
                state = 0;
                controller.driveManual(0, 0);
            } else if (joystick.dataReady) {
                controller.driveManual(-0.5f * manual_speed * joystick.y, -90.0f * manual_speed * joystick.x);
            }
            // Limit transmit rate
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            break;
    }
}

void Control::setManualMode(bool manual_state) {
    manual_enabled = manual_state;
}

bool Control::isNear(const std::string& id, const float tolerance) {
    auto point = map.points[id];
    auto rvr = rvrdata.getState();
    float distance = std::hypot(point.x - rvr.x, point.y - rvr.y);
    return distance <= tolerance;
}
