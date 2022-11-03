#pragma once
#include "boost/asio.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <optional>

/*
 * Class to read data from IMU on serial port
 * Runs in separate thread
 * Outputs roll/pitch angles and button state
 * Also scales and outputs two axes in [-1, 1]
 */
class SerialJoystick {
public:
    explicit SerialJoystick(const unsigned int baudrate) :
            io(),
            baudrate(baudrate)
    {
        background_thread = std::thread{ [&] { run(); } };
    }

    ~SerialJoystick()
    {
        // Signal to thread that we want to shut down
        stop_requested = true;
        background_thread.join();
    }

    bool connected = false;
    bool dataReady = false;

private:

    // Main loop of class, runs in background thread
    void run() {
        bool connected = false;
        std::optional<std::string> serial_message;
        std::optional<SensorData> sensordata;
        SensorData validdata {};
        std::chrono::time_point<std::chrono::steady_clock> last_filter_update;
        bool filter_valid;

        // Loop until stop is requested from parent thread
        while (true)
        {
            // --- Connect and read serial ---
            if (not connected) { // Disconnected, attempt to connect to a serial port
                if ( connectSerial() ) {
                    connected = true;
                } else {
                    // Sleep for a while, then attempt connecting again
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            } else {
                // Connected, attempt to read a line and parse it
                serial_message = readLine();
                if (serial_message.has_value()) {
                    // Message has some content, try parsing it
                    sensordata = parseMessage(serial_message.value(), ',');
                } else {
                    // Message couldn't be read, assume connection has been lost
                    connected = false;
                }
            }

            // --- Update filter ---
            if (sensordata.has_value()) {

                // Datapoint has been used, clear the value
                sensordata = {};
            }

            // --- Check if thread should keep running ---
            // If thread should finish, we can return at this point
            if (stop_requested) {
                return;
            }
        }


    }

    // Attempt to connect to the last available serial port on computer
    // Return true if successful
    bool connectSerial()
    {
        std::vector<std::string> available_ports {};
        // Iterate over possible COM ports, get first
        for (int i = 0; i < 255; i++)
        {
            try {
                std::string current_port = std::string("COM") + std::to_string(i);
                boost::asio::serial_port test_serial {io, current_port};
                available_ports.push_back(current_port);
                std::cout << "Found device at " << current_port << std::endl;
            }
            catch (std::exception& ex) {
                //std::cout << ex.what() << std::endl;
                serial = nullptr;
            }
        }

        if (available_ports.empty()) {
            std::cout << "No serial devices found" << std::endl;
            return false;
        } else {
            // Connect to last port in list
            std::cout << "Selecting last port: " << available_ports.back() << std::endl;
            serial = std::make_unique<boost::asio::serial_port>( io, available_ports.back());
            serial->set_option(baudrate);
            return true;
        }
    }

    // Try to read a line on connected serial port
    // Returns string if successful
    std::optional<std::string> readLine()
    {
        const unsigned int max_bytes = 256;
        char c;
        std::string message;
        message.reserve(max_bytes);

        for (unsigned int i = 0; i < max_bytes; i++) {
            try
            {
                // Read one byte
                // TODO: Make timeout
                serial->read_some(boost::asio::buffer(&c, 1));

                if (c == '\r') {
                    // Don't add return char to string
                    ;
                } else if (c == '\n') {
                    // Newline - end of line, message is done
                    return message;
                } else {
                    // Regular char - add to string
                    message.append(&c);
                }
            }
            catch (std::exception& ex)
            {
                std::cout << "Serial input error:" << ex.what() << std::endl;
                return std::nullopt;
            }
        }
        // No newline was received before byte limit was reached, return empty
        return std::nullopt;
    }

    // Define a data structure for storing the sensor data points
    struct Vec3 {
        float x;
        float y;
        float z;
    };
    struct IMUData {
        Vec3 gyro;
        Vec3 accel;
    };
    struct SensorData {
        IMUData imu;
        bool buttonState;
    };

    std::optional<SensorData> parseMessage(const std::string message, char separator)
    {
        SensorData data {};
        std::vector<std::string> message_parts;
        std::string element;
        // Separate message into parts based on separator character
        for (auto i = message.begin(); i <= message.end(); i++) {
            char c = *i;
            // Add part to message
            if (c == separator) {
                message_parts.emplace_back(element);
                element = {};
            } else if (i == message.end()) {
                element.append(&c);
                message_parts.emplace_back(element);
            } else {
                element.append(&c);
            }
        }
        // If message was long enough, decode
        if (message_parts.size() >= 7) {
            data.imu.accel.x = std::stof(message_parts[0]);
            data.imu.accel.y = std::stof(message_parts[1]);
            data.imu.accel.y = std::stof(message_parts[2]);
            data.imu.gyro.x = std::stof(message_parts[3]);
            data.imu.gyro.y = std::stof(message_parts[4]);
            data.imu.gyro.y = std::stof(message_parts[5]);
            data.buttonState = std::stof(message_parts[6]) > 0;
            return data;
        }
        // Read not successful, return empty
        return std::nullopt;
    }

    class Kalman {

    };

    // Fields
    boost::asio::io_service io;
    std::unique_ptr<boost::asio::serial_port> serial;
    boost::asio::serial_port_base::baud_rate baudrate;
    std::thread background_thread;
    std::atomic<bool> stop_requested = false;
};
