#include "serial_joystick.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

#include "opencv2/opencv.hpp"
#define CVUI_IMPLEMENTATION
#include "cvui.h"


int main() {
    // Variables for plotting
    const int W = 800;
    const int H = 600;
    const std::string GRAPH_WIN = "Joystick graphs";
    cvui::init(GRAPH_WIN);
    cv::Mat frame = cv::Mat(H, W, CV_8UC3);
    std::vector<double> roll(W, 0.0f);
    std::vector<double> pitch(W, 0.0f);
    unsigned int i = 0;

    auto stick = SerialJoystick(115200);


    float q = 0.15f;
    float r = 3.0f;
    stick.setKalmanQ_SD(q);
    stick.setKalmanR_SD(r);
    while (true) {
        try{
            if (stick.dataReady) {
                roll[i] = stick.roll;
                pitch[i] = stick.pitch;
                i += 1;
                i %= (unsigned int)roll.size();

                if (i%50 == 0) std::cout << "r:" << std::to_string(stick.roll) << "\t p:" << std::to_string(stick.pitch) << std::endl;
            }

            frame = cv::Mat(H, W, CV_8UC3, cv::Scalar(255, 255, 255));
            cvui::sparkline(frame, roll, 0, 0, W, H/2, 0x1F77B4);
            cvui::sparkline(frame, pitch, 0, H/2, W, H/2, 0xFF7F0E);
            if(cvui::trackbar(frame, 0, 0, 100, &q, 0.01f, 1.0f))
                stick.setKalmanQ_SD(q);
            if(cvui::trackbar(frame, 150, 0, 100, &r, 0.01f, 5.0f))
                stick.setKalmanR_SD(r);

            // Draw position of joystick
            int boxsize = 200;
            int csize = 8;
            float r_constrained = std::max(-35.0f, std::min(stick.roll, 35.0f));
            float p_constrained = std::max(-35.0f, std::min(stick.pitch, 35.0f));
            int px = boxsize/2 - csize/2 + (int) (r_constrained * ((float)(boxsize - csize) / 70.0f));
            int py = boxsize/2 - csize/2 + (int) (p_constrained * ((float)(boxsize - csize) / 70.0f));
            cvui::rect(frame, W-boxsize + 0, 0, boxsize, boxsize, 0x000000, 0xFFFFFF);
            cvui::rect(frame, W-boxsize + px, py, csize, csize, 0x000000, 0xFF0000);

            cvui::update();
            cvui::imshow(GRAPH_WIN, frame);
            // Stop if escape is pressed
            if (cv::waitKey(5)==27) {
                break;
            }
        } catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            break;
        }
    }

    return 0;
}
