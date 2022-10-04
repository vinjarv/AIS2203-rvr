#include <opencv2/opencv.hpp>
#include <zmqpp/zmqpp.hpp>

#include <vector>

/*
 * Demo - receiving a stream of encoded images over a zmq socket
 * Image is decoded with opencv and displayed
 *
 * Run with parameters - server host, port
 */

int main(int argc, char** argv){
    zmqpp::context ctx;
    zmqpp::socket sock{ctx, zmqpp::socket_type::sub};
    sock.set(zmqpp::socket_option::conflate, true); // Keep only most recent message
    sock.set(zmqpp::socket_option::subscribe, "");  // No topic

    try {
        if (argc < 3)
            throw std::exception("Too few arguments supplied, expected 'host' 'port'");
        const std::string host = argv[1];
        const std::string port = argv[2];
        const std::string endpoint = "tcp://" + host + ":" + port;
        std::cout << "Connecting to " << endpoint << std::endl;
        sock.connect(endpoint);
    } catch (const std::exception& e) {
        std::cout << e.what() << std::endl;
        return 1;
    }

    cv::namedWindow("stream");
    cv::Mat frame;

    while((cv::waitKey(1) & 0xFF) != (short)'q'){
        zmqpp::message_t msg;
        sock.receive(msg);
        std::string s{msg.get(0)};
        std::vector<uchar> raw_img{s.begin(), s.end()};
        frame = cv::imdecode(raw_img, cv::IMREAD_ANYCOLOR);
        cv::imshow("stream", frame);
    }
    return 0;
}
