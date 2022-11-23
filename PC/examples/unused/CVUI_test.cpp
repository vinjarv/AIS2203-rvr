//
// Created by MathiasM on 20.10.2022.
//
#include <opencv2/opencv.hpp>

// One (and only one) of your C++ files must define CVUI_IMPLEMENTATION
// before the inclusion of cvui.h to ensure its implementaiton is compiled.
#define CVUI_IMPLEMENTATION
#include "../cvui.h"

#define WINDOW_NAME "CVUI GUI"

int main(int argc, const char *argv[])
{
    // Create a frame where components will be rendered to.
    cv::Mat frame = cv::Mat(400, 1000, CV_8UC3);

    //cam.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    //cam.set(cv::CAP_PROP_FRAME_HEIGHT, 480);


    // Init cvui and tell it to create a OpenCV window, i.e. cv::namedWindow(WINDOW_NAME).
    cvui::init(WINDOW_NAME);
    cv::VideoCapture cap(0);
    while (true) {
        // Fill the frame with a nice color
        //frame = cv::Scalar(49, 52, 49);

        cap >> frame;

        // Render UI components to the frame
        cvui::text(frame, 10, 10, "RVR");
        cvui::window(frame, 10, 50, 200, 120, "Control");
        //TODO vvvvvMake trackbar (man/auto) instead of button!vvvvv
        cvui::button(frame, 30, 80, 150, 30, "Manual control", 0.4);
        cvui::button(frame, 30, 120, 150, 30, "Get me some coffee", 0.4);

        //Test values:
        float x_coord = 3.2;
        float y_coord = 5.4;
        int rotation_angle = 270;

        cvui::window(frame, 10, 300, 200, 120, "Location", 0.4);
        cvui::rect(frame, 20, 330, 180, 80, 0x799cb9, 0x799cb9);
        cvui::printf(frame, 30, 340, "X Coord : %.2f", x_coord);
        cvui::printf(frame, 30, 355, "Y Coord : %.2f", y_coord);
        cvui::printf(frame, 30, 370, "Angle :    %i", rotation_angle);


        // Update cvui stuff and show everything on the screen
        cvui::imshow(WINDOW_NAME, frame);

        if (cv::waitKey(20) == 27) {
            break;
        }
    }

    return 0;
}
