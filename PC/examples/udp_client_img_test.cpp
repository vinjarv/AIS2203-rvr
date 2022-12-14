
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <utility>
#include <vector>
#include <string>


//Test values:
const int img_height = 240;
const int img_width = 320;



using boost::asio::ip::udp;

const int MAX_UDP_PACKET_SIZE = 65507;
const int UDP_HEADER_PACKET_SIZE = 16;

const std::string HOST_IP = "127.0.0.1";
const int PORT = 8888;




class ImageStream {
private:
    const std::string _host_ip;
    const int _port;
    boost::asio::io_service _io_service;
    udp::socket _img_socket;
    udp::endpoint _receiver_endpoint;
    const int _default_img_height = 480;
    const int _default_img_width = 640;
    int _img_height = _default_img_height;
    int _img_width = _default_img_width;
    std::vector<std::uint8_t> _tx_buffer;
    const int _udp_header_packet_size = 16;


public:

    ImageStream(const std::string& host_ip, const int &port)
            : _host_ip(host_ip), _port(port), _img_socket(_io_service), _receiver_endpoint(boost::asio::ip::address::from_string(_host_ip), _port){
    };

    void init(){

        _img_socket.open(udp::v4());

        //udp::endpoint _receiver_endpoint(boost::asio::ip::address::from_string(_host_ip), _port);
    }

    void run() {
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
            cv::Mat img;
            img = cv::imdecode(img_bytes, 1);
            cv::imshow("Video Stream", img);

        } else {
            std::cout << "[IMAGE FRAME]_Size exception: \tFrame discarded" << std::endl;
        }
    }

    /**
     * Setters
     */
     void set_img_size(int height, int width){
        _img_height = height;
        _img_width = width;
     }
};



// Example of use
/*
int main(){
    try{
        ImageStream stream(HOST_IP, PORT);
        stream.init();
        while (true){
            stream.run();
            // Esc- button exits GUI
            if(cv::waitKey(20)==27) {
                break;
            }
        }

    }
    catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
}

*/