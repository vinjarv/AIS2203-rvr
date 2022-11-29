//
// Created by MathiasM on 03.11.2022.
//


#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <string>

//Test values:
const int img_height = 960;
const int img_width = 1280;



using boost::asio::ip::udp;

const int MAX_UDP_PACKET_SIZE = 65507;
const int UDP_HEADER_PACKET_SIZE = 16;

const std::string HOST_IP = "127.0.0.1";
const int PORT = 14;

int main(int argc, char **argv) {

    std::vector<std::uint8_t> tx_buffer = {img_height >> 8, img_height & 0xff, img_width >> 8 & 0xff, img_width & 0xff};

    try {
        //Send request of image size
        boost::asio::io_service io_service;
        udp::socket img_socket(io_service);
        img_socket.open(udp::v4());

        udp::endpoint receiver_endpoint(boost::asio::ip::address::from_string(HOST_IP), PORT);

        while (true) {

            img_socket.send_to(boost::asio::buffer(tx_buffer), receiver_endpoint);

            udp::endpoint sender_endpoint;
            std::vector<unsigned char> rx_header_buffer(UDP_HEADER_PACKET_SIZE);
            img_socket.receive_from(boost::asio::buffer(rx_header_buffer), sender_endpoint);
            int num_packets = (rx_header_buffer[0] << 8) + (rx_header_buffer[1] & 0xff);
            int packet_size = (rx_header_buffer[2] << 8) + (rx_header_buffer[3] & 0xff);
            int total_bytes = (rx_header_buffer[4] << 24) + (rx_header_buffer[5] << 16) + (rx_header_buffer[6] << 8) + (rx_header_buffer[7] & 0xff);
            int num_bytes_in_last = (rx_header_buffer[8] << 8) + (rx_header_buffer[9] & 0xff);

            std::vector<unsigned char> rx_img_buffer(packet_size);
            std::vector<std::vector<unsigned char>> img_packets_sorted(num_packets);
            std::vector<unsigned char> img_bytes;
            for (int i = 0; i < num_packets; i++) {

                auto len = img_socket.receive_from(boost::asio::buffer(rx_img_buffer), sender_endpoint);
                //get packet number
                int packet_num = (rx_img_buffer[len - 2] << 8)+(rx_img_buffer[len - 1] & 0xff);

                //resize to packet length minus 2 num bytes
                rx_img_buffer.resize(len-2);

                //struct image packets:
                img_packets_sorted[packet_num-1].insert(img_packets_sorted[packet_num-1].end(), rx_img_buffer.begin(), rx_img_buffer.end());
                rx_img_buffer.resize(packet_size);
            }

            //concatenate bytes to byte string
            for (int i = 0; i < num_packets; i++) {
                img_bytes.insert(img_bytes.end(), img_packets_sorted[i].begin(), img_packets_sorted[i].end());
            }

            //Control bytes vs header info
            if (img_bytes.size() == total_bytes-2*num_packets){
                cv::Mat img;
                img = cv::imdecode(img_bytes, 1);
                cv::imshow("Video Stream", img);
                char key = (char) cv::waitKey(25);
                if (key == 27)
                    break;
            }
            else{
                std::cout << "[IMAGE FRAME]_Size exception: \tFrame discarded" << std::endl;
            }
        }
        cv::destroyAllWindows();


    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
}