#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <optional>
#include <thread>
#include <mutex>
#include <memory>
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
            kf(std::make_unique<IMUKalman>(kf_sample_time, kf_gravity_calib))
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
        kf_q_sd = sd;
        kf->setQ_SD(kf_q_sd);
    }

    void setKalmanR_SD(const float sd) {
        kf_r_sd = sd;
        kf->setR_SD(kf_r_sd);
    }

    bool connected = false;
    bool dataReady = false;
    float roll;                     // Angle output in degrees
    float pitch;                    //
    float x;                        // Axis output - [-1, 1] - with deadzone, scaling and constraints
    float y;                        //
    float kf_gravity_calib = 9.3f;  // Default EKF parameters
    float kf_sample_time = 0.010f;
    float kf_q_sd = 1.0f;
    float kf_r_sd = 1.0f;

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
                    kf = std::make_unique<IMUKalman>(kf_sample_time, kf_gravity_calib); // Reset KF
                    setKalmanQ_SD(kf_q_sd);
                    setKalmanR_SD(kf_r_sd);
                } else {
                    std::cout << "[SerialJoystick]: Couldn't connect" << std::endl;
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
                    std::cout << "[SerialJoystick]: ReadLine failed, disconnecting" << std::endl;
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
                std::vector<float> rp = kf->run(sensordata.value());
                roll = rp[0];
                pitch = rp[1];
                dataReady = kf->samples_until_valid <= 0;
                // std::cout << sensordata.value() << std::endl;
                // Datapoint has been used, clear the value
                sensordata = {};
            }

            // Convert (r,p) angles to (x,y) output values
            updateAxisOutputs();

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
                std::cout << "[SerialJoystick]: Found device at " << current_port << std::endl;
            }
            catch (std::exception& ex) {
                //std::cout << ex.what() << std::endl;
                serial = nullptr;
            }
        }

        if (available_ports.empty()) {
            std::cout << "[SerialJoystick]: No serial devices found" << std::endl;
            return false;
        } else {
            // Connect to last port in list
            std::cout << "[SerialJoystick]: Selecting last port: " << available_ports.back() << std::endl;
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
                    std::cout << "[SerialJoystick]: Serial input error:" << ex.what() << std::endl;
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

    // Update (x,y) output axis values from roll and pitch angles
    void updateAxisOutputs(){
        const float deadzone = 5.0f;
        const float max_angle = 35.0f;
        // Guard clause - no data to process
        if (!dataReady) {
           x = 0.0f;
           y = 0.0f;
           return;
        }
        // Get angles
        x = roll;
        y = pitch;
        // If value is inside deadzone, set to 0
        if (std::abs(x) <= deadzone) {
            x = 0.0f;
        } else {
            // Either add or subtract deadzone, then scale from full range to [-1, 1]
            x = (x < 0.0f) ? (x + deadzone) : (x - deadzone);
            x /= (max_angle - deadzone);
            // Constrain to get normalized output
            x = std::max(-1.0f, std::min(x, 1.0f));
        }
        // Same as for x
        if (std::abs(y) <= deadzone) {
            y = 0.0f;
        } else {
            y = (y < 0.0f) ? (y + deadzone) : (y - deadzone);
            y /= (max_angle - deadzone);
            // Constrain to get normalized output
            y = std::max(-1.0f, std::min(y, 1.0f));
        }
    }

    class IMUKalman {
    public:
        IMUKalman(float sample_time, float gravity_calib) :
        t0(std::chrono::steady_clock::now()),
        sample_time(sample_time),
        g0(gravity_calib),
        samples_until_valid(int(3.0f / sample_time)) // 3s to run before filter output should have converged
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
            samples_until_valid = (samples_until_valid > 0) ? samples_until_valid - 1 : 0;
            return {r, p};
        }

        void setQ_SD(const float sd) {
            std::lock_guard<std::mutex> lk(m);
            Q = cv::Mat::eye(2, 2, CV_32F) * sd * sd; // Multiply by variance
        }

        void setR_SD(const float sd) {
            std::lock_guard<std::mutex> lk(m);
            R = cv::Mat::eye(3, 3, CV_32F) * sd * sd; // Multiply by variance
        }

        int samples_until_valid;

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

        [[nodiscard]] float wrapAngle(const float a) const{
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
    std::unique_ptr<IMUKalman> kf;
};
