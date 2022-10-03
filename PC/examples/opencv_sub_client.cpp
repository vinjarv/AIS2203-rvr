#include <opencv2/opencv.hpp>
#include <zmqpp/zmqpp.hpp>

#include <vector>

int main(){
    zmqpp::context ctx;
    zmqpp::socket sock{ctx, zmqpp::socket_type::sub};
    sock.set(zmqpp::socket_option::conflate, true); // Keep only
    sock.set(zmqpp::socket_option::subscribe, "");

    const std::string endpoint = "tcp://localhost:5555";
    std::cout << "Connecting to " << endpoint << std::endl;
    sock.connect(endpoint);

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
