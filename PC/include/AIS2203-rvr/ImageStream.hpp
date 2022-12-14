#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>

//#include "ArucoDetection.hpp"

class ImageStream{
public:
    ImageStream(const std::string& host_ip, const int port);
    ~ImageStream();
    void init();
    void run();

    cv::Mat getImage(){return img;};
    bool display(const std::string& window_name);
    void set_img_size(int height, int width);

private:
    void loop();
    void fetchImage();

    const std::string _host_ip;
    const int _port;
    boost::asio::io_service _io_service;
    boost::asio::ip::udp::socket _img_socket;
    boost::asio::ip::udp::endpoint _receiver_endpoint;
    const int _default_img_height = 480;
    const int _default_img_width = 640;
    int _img_height = _default_img_height;
    int _img_width = _default_img_width;
    std::vector<std::uint8_t> _tx_buffer;
    const int _udp_header_packet_size = 16;
    cv::Mat img;
    std::thread background_thread;
    std::atomic<bool> stop_requested;
    std::mutex img_mutex;
};
