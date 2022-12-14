#include <opencv2/opencv.hpp>
#include <string>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

#define GUI_NAME "GUI_v1.1"


/**
 * Class GUI
 */
class GUI {

private:
    cv::Mat _img_coffee_clear;
    cv::Mat _img_coffee_processing;
    cv::Mat _img_coffee_ready;
    std::string _coffee_state;
    int _coffee_state_var = 1;
    cv::Mat _frame;
    bool _manual_state = false;

public:
    /** Constructor that takes in a ArucoDetection object as parameter.
     *
     * @param Aruco
     */
    GUI(){}

    /** Public member function to initialize GUI with the required resources.
     *
     */
    void init(){
        _frame = cv::Mat(725, 1880, CV_8UC3);
        _frame = cv::Scalar(49, 52, 49);
        cvui::init(GUI_NAME);

        //Coffee pics
        _img_coffee_clear = cv::imread("resources/coffee_clear.jpg", cv::IMREAD_COLOR);
        _img_coffee_processing = cv::imread("resources/coffee_processing.jpg", cv::IMREAD_COLOR);
        _img_coffee_ready = cv::imread("resources/coffee_ready.jpg", cv::IMREAD_COLOR);
    }

    /** Public member function that takes in image to display, orientation of rover and a aruco flag as parameters, and
     * show a GUI window with live feed and position from rover, aruco image if identified, and a coffee interface
     * for end user.
     *
     * @param img
     * @param x_pos
     * @param y_pos
     * @param angle
     * @param flag_aruco
     */
    void run(cv::Mat& img, float x_pos, float y_pos, int angle){
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(1280, 720));
        cvui::image(_frame, 600, 0, resized_img);
        cvui::text(_frame, 610, 10, "RVR_Live", 0.5, 0x0);
        cvui::window(_frame, 5, 5, 590, 490, "CONTROL", 0.6);
        cvui::rect(_frame, 15, 45, 570, 440, 0x0e0c0d, 0x0e0c0d);

        switch(_coffee_state_var){
            case 1:
                _coffee_state = "Ready For Order";
                cvui::image(_frame, 180, 70, _img_coffee_clear);
                break;
            case 2:
                _coffee_state = "Processing";
                cvui::image(_frame, 180, 70, _img_coffee_processing);
                break;
            case 3:
                _coffee_state = "Coffee Ready";
                cvui::image(_frame, 180, 70, _img_coffee_ready);
                break;
            case 4:
                _coffee_state = "Error";
                break;
            default:
                _coffee_state = "Initializing..";
        }
        cvui::printf(_frame, 20, 50, 0.5, 0x00ff00, "%s", _coffee_state.c_str());

        if (cvui::button(_frame, 150, 440, 300, 30, "Get me some coffee", 0.6)) {
            //Get the coffee!!
            _coffee_state_var = 2;
        }

        cvui::window(_frame, 5, 500, 590, 190, "RVR STATUS", 0.6);
        cvui::rect(_frame, 15, 540, 570, 140, 0x969cba, 0x969cba);

        std::string rvr_state = "AUTO";
        cvui::printf(_frame, 30, 545, 0.5, 0xffffff, "State : %s", rvr_state.c_str());
        cvui::printf(_frame, 30, 575, 0.5, 0xffffff,"X Coord : %.2f", x_pos);
        cvui::printf(_frame, 30, 590, 0.5, 0xffffff,"Y Coord : %.2f", y_pos);
        cvui::printf(_frame, 30, 605, 0.5, 0xffffff,"Angle :    %i", angle);

        cvui::checkbox(_frame, 10, 700, "MANUAL CONTROL", &_manual_state);

        // Update and show
        cvui::imshow(GUI_NAME, _frame);

    }

    /** Public getters
     *
     */
    bool manual_ctrl() const{
        return _manual_state;
    }

};


 // EXAMPLE OF USE:


int main()
{
    cv::Mat Img;
    GUI RVR_GUI;

    RVR_GUI.init();

    //Start default camera for testing
    auto cam = cv::VideoCapture(0, cv::CAP_DSHOW);
    cam.set(cv::CAP_PROP_FRAME_WIDTH, 640);     //640
    cam.set(cv::CAP_PROP_FRAME_HEIGHT, 480);    //480
    //--------------------------------

    while (true){
        //Cap from camera (test)
        auto capture_success = cam.read(Img);

        RVR_GUI.run(Img, 3.4, 5.6, 180, false);
        if (RVR_GUI.manual_ctrl()){
            // Do something
        }

        // Esc- button exits GUI
        if(cv::waitKey(20)==27) {
            break;
        }
    }

}
*/









