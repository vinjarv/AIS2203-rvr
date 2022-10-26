#include <opencv2/opencv.hpp>
#include <zmq.h> // Requires zeromq/4.3.4

#include <vector>

/*
 * Demo - receiving a stream of encoded images over a zmq socket
 * Image is decoded with opencv and displayed
 *
 * Run with parameters - server host, port
 */

int imheight = 480;

int main(int argc, char** argv){
    auto ctx = zmq_ctx_new();
    auto socket = zmq_socket(ctx, ZMQ_DISH); // Requires zeromq with with_draft_api build option
    zmq_join(socket, "test");

    bool conflate_on = true;
    zmq_setsockopt(socket, ZMQ_CONFLATE, &conflate_on, 1);

    try {
        if (argc < 3)
            throw std::exception("Too few arguments supplied, expected 'host' 'port'");
        const std::string host = argv[1];
        const std::string port = argv[2];
        const std::string endpoint = "tcp://" + host + ":" + port;
        std::cout << "Connecting to " << endpoint << std::endl;
        zmq_connect(socket, endpoint.c_str());
    } catch (const std::exception& e) {
        std::cout << e.what() << std::endl;
        return 1;
    }

    cv::namedWindow("stream");
    cv::Mat frame;

    while((cv::waitKey(1) & 0xFF) != (short)'q'){
        zmq_msg_t msg;
        zmq_msg_init(&msg);
        zmq_recvmsg(socket, &msg, 0);

        auto data_ptr = (uchar*) zmq_msg_data(&msg);
        auto data_size = zmq_msg_size(&msg);

        std::cout << "Message received" << std::endl;
        std::cout << "Size " << data_size << std::endl;

        if (data_size > 0) {

            std::vector<uchar> raw_img{data_ptr, data_ptr + data_size};
            frame = cv::imdecode(raw_img, cv::IMREAD_ANYCOLOR);
            std::cout << frame.size << std::endl;
            float aspectRatio = (float) frame.cols / (float) frame.rows;
            int imwidth = (int) ((float) imheight * aspectRatio); // Automatically resize based on specified width
            cv::Mat resized = {cv::Size(imwidth, imheight), frame.type()};
            cv::resize(frame, resized, resized.size());
            cv::imshow("stream", resized);
        }
        zmq_msg_close(&msg);
    }
    return 0;
}
