#include "ImageStream.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

//Test values:
const int img_height = 240;
const int img_width = 320;

using boost::asio::ip::udp;

//const int MAX_UDP_PACKET_SIZE = 65507;
//const int UDP_HEADER_PACKET_SIZE = 16;

//const std::string HOST_IP = "127.0.0.1";
const std::string HOST_IP = "10.25.45.75";
const int PORT = 8889;

int main(){
    try{
        ImageStream stream(HOST_IP, PORT);
        stream.init();
        stream.set_img_size(img_height, img_width);
        stream.run();
        while (stream.display("UDP stream")) {};
    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
}
