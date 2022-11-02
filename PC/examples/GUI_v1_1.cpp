
//
// Created by MathiasM on 20.10.2022.
//
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

#define GUI_NAME "GUI_v1.1"

int main(int argc, const char *argv[])
{
    // Create GUI- frame
    cv::Mat Frame = cv::Mat(725, 1880, CV_8UC3);

    // Create openCV window
    cvui::init(GUI_NAME);

    // Matrix to put the captured image in
    cv::Mat Img{};
    // Matrix with picture in it
    cv::Mat img_coffee_clear = cv::imread("resources/coffee_clear.jpg", cv::IMREAD_COLOR);
    cv::Mat img_coffee_processing = cv::imread("resources/coffee_processing.jpg", cv::IMREAD_COLOR);
    cv::Mat img_coffee_ready = cv::imread("resources/coffee_ready.jpg", cv::IMREAD_COLOR);



    // Start default camera for testing
    auto cam = cv::VideoCapture(0, cv::CAP_DSHOW);
    cam.set(cv::CAP_PROP_FRAME_WIDTH, 1280);     //640
    cam.set(cv::CAP_PROP_FRAME_HEIGHT, 720);    //480


    while (true){
        Frame = cv::Scalar(49, 52, 49);
        auto capture_success = cam.read(Img);
        if (capture_success) {
            cvui::image(Frame, 600, 0, Img);
            cvui::text(Frame, 610, 10, "RVR_Live", 0.5, 0x0);
        }

        cvui::window(Frame, 5, 5, 590, 490, "CONTROL", 0.6);
        cvui::rect(Frame, 15, 45, 570, 440, 0x0e0c0d, 0x0e0c0d);
        int coffee_state_var = 1;
        std::string coffee_state;
        switch(coffee_state_var){
            case 1: coffee_state = "Ready For Order";
                break;
            case 2: coffee_state = "Processing";
                break;
            case 3: coffee_state = "Coffee Ready";
                break;
            case 4: coffee_state = "Error";
                break;
            default: coffee_state = "Initializing..";
        }
        cvui::printf(Frame, 20, 50, 0.5, 0x00ff00, "%s", coffee_state.c_str());
        cvui::image(Frame, 180, 70, img_coffee_ready);
        if (cvui::button(Frame, 150, 440, 300, 30, "Get me some coffee", 0.6)){
            //Get the coffee!!
            std::cout<<"Coffee on its way"<<'\n';
        }

        cvui::window(Frame, 5, 500, 590, 190, "RVR STATUS", 0.6);
        cvui::rect(Frame, 15, 540, 570, 140, 0x969cba, 0x969cba);
        // Status test values
        float x_coord = 3.2;
        float y_coord = 5.4;
        int rotation_angle = 270;
        std::string rvr_state = "AUTO";
        cvui::printf(Frame, 30, 545, 0.5, 0xffffff, "State : %s", rvr_state.c_str());
        cvui::printf(Frame, 30, 575, 0.5, 0xffffff,"X Coord : %.2f", x_coord);
        cvui::printf(Frame, 30, 590, 0.5, 0xffffff,"Y Coord : %.2f", y_coord);
        cvui::printf(Frame, 30, 605, 0.5, 0xffffff,"Angle :    %i", rotation_angle);

        bool MANUAL_STATE = false;
        cvui::checkbox(Frame, 10, 700, "MANUAL CONTROL", &MANUAL_STATE);







        // Update and show
        cvui::imshow(GUI_NAME, Frame);

        // Esc- button exits GUI
        if(cv::waitKey(20)==27) {
            break;

        }
    }
    return 0;
}
