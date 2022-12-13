#include <algorithm>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>
#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include "rvr_controller.hpp"
#include "serial_joystick.hpp"

int main() {
    // Variables for plotting
    const int W = 200;
    const int H = 220;
    const float update_rate = 30.0f;
    const std::string GRAPH_WIN = "Joystick position";
    cvui::init(GRAPH_WIN);
    cv::Mat frame = cv::Mat(H, W, CV_8UC3);

    auto stick = SerialJoystick(115200);
    auto controller = RVRController("10.25.45.75", 5555);

    const float servo_vel = 360.0f;      // Degrees/s
    const float servo0_home = 145.0f;
    const float servo1_home = 90.0f;
    float servo0_pos = 0.0f;
    float servo1_pos = 0.0f;

    auto t_prev = std::chrono::steady_clock::now();
    bool manual_enabled = false;
    bool servo_enabled = false;
    float q = 0.4f;
    float r = 4.0f;
    stick.setKalmanQ_SD(q);
    stick.setKalmanR_SD(r);

    // Wait for connection
    std::this_thread::sleep_for(std::chrono::milliseconds (200));
    controller.setServo(0, servo0_home);
    controller.setServo(1, servo1_home);

    while (true) {
        t_prev = std::chrono::steady_clock::now();
        try{
            frame = cv::Mat(H, W, CV_8UC3, cv::Scalar(255, 255, 255));
            // Draw position of joystick
            int boxsize = 200;
            int csize = 8;
            float r_constrained = std::clamp(stick.x, -1.0f, 1.0f);
            float p_constrained = std::clamp(stick.y, -1.0f, 1.0f);
            int px = boxsize/2 - csize/2 + (int) (r_constrained * ((float)(boxsize - csize) / 2.0f));
            int py = boxsize/2 - csize/2 + (int) (p_constrained * ((float)(boxsize - csize) / 2.0f));
            cvui::rect(frame, W-boxsize + 0, 0, boxsize, boxsize, 0x000000, 0xFFFFFF);
            cvui::rect(frame, W-boxsize + px, py, csize, csize, 0x000000, (stick.button ? 0x00FF00 : 0xFF0000));

            cvui::checkbox(frame, 2, 202, "Manual on", &manual_enabled, 0x000000);
            cvui::checkbox(frame, 100, 202, "Servo", &servo_enabled, 0x000000);

            cvui::update();
            cvui::imshow(GRAPH_WIN, frame);
            // Stop if escape is pressed
            if (cv::waitKey(5)==27) {
                break;
            }

            // Send command to RVR
            if (stick.dataReady && (stick.x != 0.0f || stick.y != 0.0f) && manual_enabled)
            {
                if ( stick.button && servo_enabled ) {
                    controller.driveStop();
                    servo0_pos -= servo_vel * 1.0f/update_rate * stick.x;
                    servo1_pos -= servo_vel * 1.0f/update_rate * stick.y;
                    servo0_pos = std::clamp(servo0_pos, -90.0f, 90.0f);
                    servo1_pos = std::clamp(servo1_pos, -45.0f, 45.0f);
                    controller.setServo(0, servo0_home + servo0_pos);
                    controller.setServo(1, servo1_home + servo1_pos);
                } else if (stick.button) {
                    controller.driveManual(-3.0f * stick.y, -270.0f * stick.x);
                } else {
                    controller.driveManual(-0.5f * stick.y, -90.0f * stick.x);
                }
            } else {
                controller.driveStop();
            }

            // Color status display
            if ( !manual_enabled )
                controller.setColour(0, 255, 0); // Green
            else if( stick.button && servo_enabled )
                controller.setColour(255, 250, 70); // Yellow
            else if( stick.button && !servo_enabled )
                controller.setColour(255, 0, 0); // Red
            else if ( !stick.button && manual_enabled )
                controller.setColour(255, 70, 215); // Pink

        } catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            break;
        }

        // Wait for fixed update rate
        std::this_thread::sleep_until(t_prev + std::chrono::microseconds ( (long) (1e6  / update_rate) ));
    }

    // Try to stop RVR before disconnecting
    controller.driveStop();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return 0;
}
