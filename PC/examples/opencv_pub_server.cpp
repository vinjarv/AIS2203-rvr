#include <opencv2/opencv.hpp>
#include <zmqpp/zmqpp.hpp>

#include <vector>
#include <thread>

int main(){
    zmqpp::context ctx;
    zmqpp::socket sock{ctx, zmqpp::socket_type::pub};

    const std::string endpoint = "tcp://*:5555";
    std::cout << "Server started, listening on " << endpoint << std::endl;
    sock.bind(endpoint);

    auto cam = cv::VideoCapture(0, cv::CAP_DSHOW);
    cam.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cam.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cv::Mat img{};

    while(true){
        auto capture_success = cam.read(img);
        if (capture_success) {
            cv::Mat grey;
            cv::cvtColor(img, grey, cv::COLOR_BGR2GRAY);
            std::vector<uchar> img_file{};
            zmqpp::message_t msg;
            cv::imencode(".jpg", grey, img_file);
            std::string img_str{img_file.begin(), img_file.end()};
            msg << img_str;
            sock.send(msg, false);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
