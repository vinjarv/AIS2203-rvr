#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <optional>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "boost/asio.hpp"
#include "opencv2/opencv.hpp"

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
            baudrate(baudrate),
            kf(0.010f, 10.3f)
    {
        background_thread = std::thread{ [&] { run(); } };
    }

    ~SerialJoystick()
    {
        // Signal to thread that we want to shut down
        stop_requested = true;
        background_thread.join();
    }

    void setKalmanQ_SD(const float sd) {
        kf.setQ_SD(sd);
    }

    void setKalmanR_SD(const float sd) {
        kf.setR_SD(sd);
    }

    bool connected = false;
    bool dataReady = false;
    float roll;
    float pitch;
private:

    // Main loop of class, runs in background thread
    void run() {
        std::optional<std::string> serial_message;
        std::optional<SensorData> sensordata;
        SensorData validdata {};
        std::chrono::time_point<std::chrono::steady_clock> last_filter_update = std::chrono::steady_clock::now();
        bool filter_valid;

        // Loop until stop is requested from parent thread
        while (true)
        {
            // --- Connect and read serial ---
            if (! connected) { // Disconnected, attempt to connect to a serial port
                dataReady = false;
                if ( connectSerial() ) {
                    readLine(1500); // Read and discard first line
                    connected = true;
                } else {
                    std::cout << "Couldn't connect" << std::endl;
                    // Sleep for a while, then attempt connecting again
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            } else {
                // Connected, attempt to read a line and parse it
                serial_message = readLine(500);
                if (serial_message.has_value()) {
                    // Message has some content, try parsing it
                    sensordata = parseMessage(serial_message.value(), ',');
                } else {
                    // Message couldn't be read, assume connection has been lost
                    std::cout << "ReadLine failed, disconnecting" << std::endl;
                    connected = false;
                }
            }

            // --- Update filter ---
            if (sensordata.has_value()) {
//                auto now = std::chrono::steady_clock::now();
//                auto dt_micros = std::chrono::duration_cast<std::chrono::microseconds>(now - last_filter_update).count();
//                float dt_millis = (float) dt_micros / 1000.0f;
//                last_filter_update = now;
//                std::cout << dt_millis << "\n";
                std::vector<float> rp = kf.run(sensordata.value());
                roll = rp[0];
                pitch = rp[1];
                dataReady = true;
                // std::cout << sensordata.value() << std::endl;
                // Datapoint has been used, clear the value
                sensordata = {};
            }

            // --- Check if thread should keep running ---
            // If thread should finish, we can return at this point
            if (stop_requested) {
                dataReady = false;
                connected = false;
                return;
            }
        }
    }

    // Attempt to connect to the -last available serial port on computer
    // Ex: selects port COM5 if available ports are [COM0, COM4, COM5]
    // Returns true if successful
    bool connectSerial()
    {
        std::vector<std::string> available_ports {};
        // Iterate over possible COM ports, get first
        for (int i = 0; i < 64; i++)
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
    // Read timeout in milliseconds
    // Returns string if successful
    std::optional<std::string> readLine(const unsigned int timeout)
    {
        const unsigned int max_bytes = 256;
        char c;
        std::string message {};
        message.reserve(max_bytes);

        // Boost.Asio doesn't support timeouts for the serial port read method
        // We want to handle disconnections, so this read loop should run in parallel with a timeout check
        // Here we will use the condition variable method wait_for to handle the timeout
        // The read must then happen in a new thread
        std::mutex m;
        std::condition_variable cv;
        bool read_finished = false;
        bool read_success = false;

        std::thread t {[&]
        {
            for (unsigned int i = 0; i < max_bytes; i++) {
                try
                {
                    // Read one byte
                    serial->read_some(boost::asio::buffer(&c, 1));

                    if (c == '\n') {
                        // Newline - end of line, message is complete
                        read_success = true;
                        break; // Jump out of for-loop {
                    } else if (c == '\r') {
                        // Don't add return char to string
                        ;
                    } else {
                        // Regular char - add to string
                        message.append(&c, 1);
                    }
                }
                catch (std::exception& ex)
                {
                    std::cout << "Serial input error:" << ex.what() << std::endl;
                    break; // Jump out of for-loop
                }
            }
            // If loop finished without reading a newline, message was too long
            read_finished = true;
            cv.notify_one();
        }
        };

        // Wait for thread finish or timeout
        std::unique_lock<std::mutex> lk {m};
        if (cv.wait_for(lk, std::chrono::milliseconds(timeout), [&]{return read_finished;})) {
            // Read completed
            // Check if message was received successfully
            t.join();
            return read_success ? std::optional{message} : std::nullopt;
        } else {
            // Timed out
            t.join();
            return std::nullopt;
        }
    }

    // Define a data structure for storing the sensor data points

    struct SensorData {

        struct IMUData {

            struct Vec3 {
                float x;
                float y;
                float z;
            };

            Vec3 gyro;
            Vec3 accel;
        };

        IMUData imu;
        bool buttonState;

        friend std::ostream& operator << (std::ostream& os, const SensorData& s){
            os << "Accel. XYZ:\t" << s.imu.accel.x << "\t" << s.imu.accel.y << "\t" << s.imu.accel.z << std::endl;
            os << "Gyro. XYZ:\t" << s.imu.gyro.x << "\t" << s.imu.gyro.y << "\t" << s.imu.gyro.z << std::endl;
            os << "Button state:\t" << s.buttonState << " ";
            return os;
        }

    };

    std::optional<SensorData> parseMessage(const std::string message, char separator)
    {

        SensorData data {};
        std::vector<std::string> message_parts;
        std::string element{};
        // Separate message into parts based on separator character
        for (auto i = message.begin(); i <= message.end(); i++) {
            if (i == message.end()) {
                message_parts.emplace_back(element);
                break;
            }
            char c = *i;
            if (c == separator) {
                message_parts.emplace_back(element);
                element = {};
            } else {
                // Check if char is ASCII numeric or decimal separator (.) or -
                if ((c >= 0x30 && c <= 0x39) || c == 0x2E || c == 0x2D)
                    element.append(&c, 1);
            }
        }
        // If message was long enough, decode
        if (message_parts.size() == 7) {
            data.imu.accel.x = std::stof(message_parts[0]);
            data.imu.accel.y = std::stof(message_parts[1]);
            data.imu.accel.z = std::stof(message_parts[2]);
            data.imu.gyro.x = std::stof(message_parts[3]);
            data.imu.gyro.y = std::stof(message_parts[4]);
            data.imu.gyro.z = std::stof(message_parts[5]);
            data.buttonState = std::stof(message_parts[6]) > 0;
            return data;
        }
        // Read not successful, return empty
        return std::nullopt;
    }

    class IMUKalman {
    public:
        IMUKalman(float sample_time, float gravity_calib) :
        t0(std::chrono::steady_clock::now()),
        sample_time(sample_time),
        g0(gravity_calib)
        {
            x0 = cv::Mat::zeros(2, 1, CV_32F);
            P = cv::Mat::zeros(2, 2, CV_32F);
            setQ_SD(1);
            setR_SD(1);
        }

        std::vector<float> run(const SensorData data)
        {
            predict(data);
            correct(data);
            const float r = x0.at<float>(0) * RAD_TO_DEG;
            const float p = x0.at<float>(1) * RAD_TO_DEG;
            return {r, p};
        }

        void setQ_SD(const float sd) {
            std::lock_guard<std::mutex> lk(m);
            Q = cv::Mat::eye(2, 2, CV_32F) * sd;
        }

        void setR_SD(const float sd) {
            std::lock_guard<std::mutex> lk(m);
            R = cv::Mat::eye(3, 3, CV_32F) * sd;
        }

    private:
        void predict(const SensorData data)
        {
            // Unused: serial read rate was not constant, so we're assuming the arduinos fixed sample rate is accurate enough
            //            // Get timestep length
            //            auto now = std::chrono::steady_clock::now();
            //            auto dt_micros = std::chrono::duration_cast<std::chrono::microseconds>(now - t0).count();
            //            float dt = (float) dt_micros * 1e-6;
            //            t0 = now;
            //            dt = std::max(0.0f, std::min(dt, 0.1f)); // Constrain
            float phi = x0.at<float>(0);
            float theta = x0.at<float>(1);
            float sp = std::sin(phi); float cp = std::cos(phi);
            float st = std::sin(theta); float ct = std::cos(theta); float tt = std::tan(theta);
            float p = data.imu.gyro.x;
            float q = data.imu.gyro.y;
            float r = data.imu.gyro.z;
            // Propagation model for gyro sensor
            cv::Mat f = (cv::Mat_<float>(2, 1)
                    <<  p + q*sp*tt + r*cp*tt,
                        q*cp - r*sp
            );
            // Jacobian of f
            cv::Mat A = (cv::Mat_<float>(2, 2)
                    <<  (q*cp - r*sp)*tt,  (q*sp + r*cp)/(ct*ct),
                        -q*sp - r*cp,      0
            );
            // Euler integration
            x0 = x0 + f*sample_time;
            // We want to use Q, lock mutex
            std::lock_guard<std::mutex> lk(m);
            // Update error covariance
            P = P + (A*P + P*A.t() + Q) * sample_time;
            //A = A * sample_time;
            //P = A*P*A.t() + Q*sample_time;

            // Constrain angles to [-pi, pi]
            x0.at<float>(0) = wrapAngle(x0.at<float>(0));
            x0.at<float>(1) = wrapAngle(x0.at<float>(1));
            // Euler angles have a singularity problem at pitch +- 90 deg, constrain to avoid
            x0.at<float>(1) = std::max(-PI/2.0f + 0.05f, std::min(x0.at<float>(1), PI/2.0f -0.05f));
        }

        void correct(SensorData data)
        {
            float phi = x0.at<float>(0);
            float theta = x0.at<float>(1);
            float sp = std::sin(phi); float cp = std::cos(phi);
            float st = std::sin(theta); float ct = std::cos(theta);
            // Measurement model
            cv::Mat h = (cv::Mat_<float>(3, 1)
                    <<  +g0*st,
                        -g0*ct*sp,
                        -g0*ct*cp
            );

            cv::Mat C = (cv::Mat_<float>(3, 2)
                    <<   0,        g0*ct,
                        -g0*ct*cp, g0*st*sp,
                        +g0*ct*sp, g0*st*cp
            );
            cv::Mat z = (cv::Mat_<float>(3,1) << data.imu.accel.x, data.imu.accel.y, data.imu.accel.z);

            // We want to use R, lock mutex
            std::lock_guard<std::mutex> lk(m);
            // Calculate Kalman gain, update states and error covariance
            cv::Mat Kk = P*C.t() * (C*P*C.t() + R).inv();
            x0 = x0 + Kk*(z-h);
            P = P - Kk*C*P;
            // Constrain angles to [-pi, pi]
            x0.at<float>(0) = wrapAngle(x0.at<float>(0));
            x0.at<float>(1) = wrapAngle(x0.at<float>(1));
            // Euler angles have a singularity problem at pitch +- 90 deg, constrain to avoid
            x0.at<float>(1) = std::max(-PI/2.0f + 0.05f, std::min(x0.at<float>(1), PI/2.0f -0.05f));
        }

        float wrapAngle(const float a){
            // Mod 2*pi
            float times = std::floor(std::abs(a / (2*PI)));
            float sign = 1 - 2*(float)std::signbit(a);
            float amod = a - times*PI*sign;
            // Constrain to [-pi, pi]
            if (amod < -PI) {
                amod = amod + 2*PI;
            } else if (amod > PI) {
                amod = amod - 2*PI;
            }
            return amod;
        }

        std::mutex m;
        cv::Mat x0; // State variables
        cv::Mat P;
        cv::Mat Q;
        cv::Mat R;
        std::chrono::time_point<std::chrono::steady_clock> t0;
        const float sample_time;
        const float g0;
        const float RAD_TO_DEG = 180.0f / 3.14159265358979f;
        const float PI = 3.14159265358979f;
    };

    // Fields
    boost::asio::io_service io;
    std::unique_ptr<boost::asio::serial_port> serial;
    boost::asio::serial_port_base::baud_rate baudrate;
    std::thread background_thread;
    std::atomic<bool> stop_requested = false;
    IMUKalman kf;
};
