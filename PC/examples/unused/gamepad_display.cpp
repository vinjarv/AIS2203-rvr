#include <iostream>
#include <cmath>
#include <vector>

#include "Windows.h"
#include "Xinput.h"
#include "opencv2/opencv.hpp"

/*
 * Testing Xbox gamepad functionality with Xinput.h
 * Display joystick and trigger positions with an OpenCV GUI
 */

int main(){
    XINPUT_STATE state = {};
    cv::namedWindow("Gamepad state");

    // Rumble test
    bool done = false;
    while((!done) && ((cv::waitKey(1)&0xFF) != 'q')){

        // Try fetching state of first connected controller
        double result = XInputGetState(0, &state);
        if (result == 0x0){ // ERROR_SUCCESS (0x0) - read successful

            float Lx_norm = (float)state.Gamepad.sThumbLX / SHRT_MAX;
            float Ly_norm = (float)state.Gamepad.sThumbLY / SHRT_MAX;
            bool L_pressed = state.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_THUMB;
            bool R_pressed = state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB;
            float Rx_norm = (float)state.Gamepad.sThumbRX / SHRT_MAX;
            float Ry_norm = (float)state.Gamepad.sThumbRY / SHRT_MAX;
            // Draw window
            cv::Mat canvas = cv::Mat::zeros(300, 400, CV_32FC3);
            // Left thumbstick
            cv::Point Lpos = {50 +100/2 + int(Lx_norm*50), 50 +100/2 - int(Ly_norm*50)};
            cv::rectangle(canvas, cv::Rect{50, 50, 100, 100}, cv::Scalar(1, 0, 0), 2);
            cv::line(canvas, cv::Point(50 +100/2, 50 +100/2), Lpos, cv::Scalar(0, 0, 1), 2);
            cv::circle(canvas, Lpos, 10, cv::Scalar(0, (1-((float)!L_pressed)), (1-(float)L_pressed)), -1);
            // Right thumbstick
            cv::Point Rpos = {250 +100/2 + int(Rx_norm*50), 50 +100/2 - int(Ry_norm*50)};
            cv::rectangle(canvas, cv::Rect{250, 50, 100, 100}, cv::Scalar(1, 0, 0), 2);
            cv::line(canvas, cv::Point(250 +100/2, 50 +100/2), Rpos, cv::Scalar(0, 0, 1), 2);
            cv::circle(canvas, Rpos, 10, cv::Scalar(0, (1-((float)!R_pressed)), (1-(float)R_pressed)), -1);

            // Left triggers
            bool Lbump = state.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER;
            float Ltrig = (float)state.Gamepad.bLeftTrigger /  UCHAR_MAX;
            cv::rectangle(canvas, cv::Rect{50, 30, 100, 15}, cv::Scalar(0, (1-(float)Lbump), (float)Lbump), -1);
            cv::rectangle(canvas, cv::Rect{50, 10, 100, 15}, cv::Scalar(0, (1-Ltrig), Ltrig), -1);
            // Right triggers
            bool Rbump = state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER;
            float Rtrig = (float)state.Gamepad.bRightTrigger /  UCHAR_MAX;
            cv::rectangle(canvas, cv::Rect{250, 30, 100, 15}, cv::Scalar(0, (1-(float)Rbump), (float)Rbump), -1);
            cv::rectangle(canvas, cv::Rect{250, 10, 100, 15}, cv::Scalar(0, (1-Rtrig), Rtrig), -1);

            cv::putText(canvas, "Hello world", cv::Point{50, 250}, cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(1, 1, 1));

            cv::imshow("Gamepad state", canvas);
        }else{ // On disconnect
            done = true;
        }
    }

    return 0;
}
