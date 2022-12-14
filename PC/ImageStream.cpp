#include "ImageStream.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>


using boost::asio::ip::udp;

ImageStream::ImageStream(const std::string& host_ip, const int port)
        : _host_ip(host_ip),
        _port(port),
        _img_socket(_io_service),
        _receiver_endpoint(boost::asio::ip::address::from_string(_host_ip), _port)
        {};

ImageStream::~ImageStream() {
    stop_requested = true;
    background_thread.join();
}

void ImageStream::init(){
    _img_socket.open(udp::v4());
}

void ImageStream::run() {
    if (background_thread.joinable())
        return;
    background_thread = std::thread([&]{loop();});
}

void ImageStream::loop() {
    while (!stop_requested) {
        fetchImage();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void ImageStream::fetchImage(){
    // Clear incoming packets
    while (_img_socket.available() > 0) {
        std::vector<unsigned char> discard(_img_socket.available());
        _img_socket.receive_from(boost::asio::buffer(discard), _receiver_endpoint);
    }
    // Try to receive a frame and store it in img
    try
    {
        _tx_buffer = {static_cast<unsigned char>(_img_height >> 8),
                      static_cast<unsigned char>(_img_height & 0xff),
                      static_cast<unsigned char>(_img_width >> 8 & 0xff),
                      static_cast<unsigned char>(_img_width & 0xff)};

        _img_socket.send_to(boost::asio::buffer(_tx_buffer), _receiver_endpoint);

        udp::endpoint sender_endpoint;
        std::vector<unsigned char> rx_header_buffer(_udp_header_packet_size);
        _img_socket.receive_from(boost::asio::buffer(rx_header_buffer), sender_endpoint);
        int num_packets = (rx_header_buffer[0] << 8) + (rx_header_buffer[1] & 0xff);
        int packet_size = (rx_header_buffer[2] << 8) + (rx_header_buffer[3] & 0xff);
        int total_bytes = (rx_header_buffer[4] << 24) + (rx_header_buffer[5] << 16) + (rx_header_buffer[6] << 8) +
                          (rx_header_buffer[7] & 0xff);
        int num_bytes_in_last = (rx_header_buffer[8] << 8) + (rx_header_buffer[9] & 0xff);

        std::vector<unsigned char> rx_img_buffer(packet_size);
        std::vector<std::vector<unsigned char>> img_packets_sorted(num_packets);
        std::vector<unsigned char> img_bytes;
        for (int i = 0; i < num_packets; i++) {

            auto len = _img_socket.receive_from(boost::asio::buffer(rx_img_buffer), sender_endpoint);
            //get packet number
            int packet_num = (rx_img_buffer[len - 2] << 8) + (rx_img_buffer[len - 1] & 0xff);

            //resize to packet length minus 2 num bytes
            rx_img_buffer.resize(len - 2);

            //struct image packets:
            img_packets_sorted[packet_num - 1].insert(img_packets_sorted[packet_num - 1].end(), rx_img_buffer.begin(),
                                                      rx_img_buffer.end());
            rx_img_buffer.resize(packet_size);
        }

        //concatenate bytes to byte string
        for (int i = 0; i < num_packets; i++) {
            img_bytes.insert(img_bytes.end(), img_packets_sorted[i].begin(), img_packets_sorted[i].end());
        }
        //Control bytes vs header info
        if (img_bytes.size() == total_bytes - 2 * num_packets) {
            std::lock_guard<std::mutex> lk(img_mutex);
            img = cv::imdecode(img_bytes, 1);
        } else {
            std::cout << "[IMAGE FRAME]_Size exception: \tFrame discarded" << std::endl;
        }
    }
    catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
    }
}

bool ImageStream::display(const std::string &window_name) {
    {
        std::lock_guard<std::mutex> lk(img_mutex);
        if (img.empty())
            return true;
        cv::Mat resized;
        cv::resize(img, resized, cv::Size(_img_width*2, _img_height*2));
        cv::imshow(window_name, resized);
    }
    // Esc- button exits GUI
    if(cv::waitKey(1)==27) {
        return false;
    }
    return true;
}

/**
 * Setters
 */
void ImageStream::set_img_size(int height, int width) {
    _img_height = height;
    _img_width = width;
}
