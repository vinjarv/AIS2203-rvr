#include <opencv2/opencv.hpp>
#include <zmq.h> // Requires zeromq/4.3.4

#include <vector>
#include <thread>


int main(){
    auto ctx = zmq_ctx_new();
    auto socket = zmq_socket(ctx, ZMQ_RADIO); // Requires zeromq with with_draft_api build option

    const std::string endpoint = "tcp://*:5555";
    std::cout << "Server started, listening on " << endpoint << std::endl;
    zmq_bind(socket, endpoint.c_str());

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
            cv::imencode(".jpg", img, img_file);

            zmq_msg_t msg;
            zmq_msg_init_size(&msg, img_file.size());
            memcpy(zmq_msg_data(&msg), img_file.data(), img_file.size());
            zmq_msg_set_group(&msg, "test");
            std::cout << "Message sent, size " << zmq_msg_size(&msg) << std::endl;
            zmq_sendmsg(socket, &msg, ZMQ_DONTWAIT);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
