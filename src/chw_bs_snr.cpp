
#include "../include/chw_bs_snr.hpp"
#include "../include/utility.hpp"

// #include "std_msgs/msg/float32.hpp"

// #include <boost/range/adaptors.hpp>
// #include <boost/filesystem.hpp>


#include <time.h>

#include <functional>
// #include <iostream>
// #include <cstdlib>
// #include <fstream>
// #include <sstream>
#include <chrono>
// #include <cstdio>
// #include <limits>
#include <memory>
// #include <thread>
// #include <cmath>
#include <unistd.h>


// #define BOOST_RANGE_ENABLE_CONCEPT_ASSERT 0

using namespace ytcg;
using namespace std::chrono;

// using std::placeholders::_1;
using std::string;

// namespace bfs = boost::filesystem;
// namespace ba = boost::adaptors;

// BsSnrPublisher::BsSnrPublisher() : BsSnrPublisher::BsSnrPublisher(CommProtocol::_80211) { }

// class SnrLogger : public rclcpp::Node
// SnrLogger::SnrLogger()
// {
    // public:
        SnrLogger::SnrLogger()
        : Node("snr_logger"), count_(0)
        {
            // publisher_ = this->create_publisher<std_msgs::msg::Float32>(SNR_TOPIC, 10);
            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(CHW_BS_SNR_RATE_MS), std::bind(&SnrLogger::snr_callback, this));

            rclcpp::on_shutdown(std::bind(&SnrLogger::shutdown_callback, this));
        }
    
    // private:
        // void snr_callback(void) {
void SnrLogger::snr_callback(void) {

            static size_t _count = 0;
            int rx_pos;
            uint64_t micro_se;
            char buffer[100];
            string cmd, output, output_;
            // msg_.data.clear();

            // communication channel between chw---bs uses LoRa
            if (count__ == 0) {
                // LoRa needs to be initialized... Also testing if all is
                // okay the first time with LoRa (similarly as with 802.11)
                fd_ = utility_serial_open(DEF_PORT_READ);
                utility_serial_write(fd_, "sys factoryRESET\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
                std::this_thread::sleep_for(std::chrono::milliseconds(PAUSE_SYSRESET_RN2903));
                output_ = utility_serial_read(fd_, "sys get ver\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
                RCLCPP_INFO(this->get_logger(), "%s", output_.c_str());
                utility_serial_write(fd_, "mac pause\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
                utility_serial_write(fd_, "radio set pwr 10\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
                utility_serial_write(fd_, "radio set mod fsk\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
                output_ = utility_serial_read(fd_, "radio rx 0\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
                RCLCPP_INFO(this->get_logger(), "%s", output_.c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(PAUSE_FIRST_RX_RN2903));
        
                if (output_.find("ok") != std::string::npos) {
                    RCLCPP_INFO(this->get_logger(), "Established connection with the base-station via LoRa");
                } else {
                    RCLCPP_FATAL(this->get_logger(), "No connection to the base-station utilizing LoRa");
                    return;
                }
            }

            try {
                // try reading from the serial port
                output_ = utility_serial_read(fd_, "radio rx 0\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
                RCLCPP_INFO(this->get_logger(), "%s", output_.c_str());
         
                if ((rx_pos = output_.find("radio_rx")) != std::string::npos) {
                    utility_serial_write(fd_, "sys set pindig GPIO10 1\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Unable to retrieve data from the channel with LoRa this time");
                    return;
                }
        
                // utility_serial_write(fd_, "sys set pindig GPIO10 0\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
                // output = "x:" + std::to_string((int)std::stoul(output_.substr(rx_pos+10,2), nullptr, 16) - 100) + " " +
                // "y:" + std::to_string((int)std::stoul(output_.substr(rx_pos+12,2), nullptr, 16) - 100);
            } catch (...) { 
                RCLCPP_FATAL(this->get_logger(), "Cannot communicate with the device");
                return;
            }

            if (_count++ == 0) {
                __LOG_SNR::file().open(LOG_SNR_FILE, std::ios::app);
                RCLCPP_INFO_STREAM(this->get_logger(), "logging enabled, check file " << LOG_SNR_FILE);
                snprintf(buffer, sizeof(buffer), "time, SNR");
                __LOG_SNR::file() << buffer << std::endl;
            }

            snr = 69;

            output_ = utility_serial_read(fd_, "radio get snr\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
            RCLCPP_WARN(this->get_logger(), "snr command output %s", output_.c_str());

            char buffer_[10];
            int size_bytes = read(fd_, &buffer_, sizeof(buffer_));

            if (size_bytes < 0) {
                RCLCPP_WARN(this->get_logger(), "No bytes returned in output!");
            } else {
                try {
                    snr = atof(buffer_);
                    // snr = atof(output__.c_str());
                } catch (...) {
                    RCLCPP_ERROR(this->get_logger(), "Unable to read SNR");
                }
            }

            

            RCLCPP_WARN(this->get_logger(), "SNR f: %f", snr); 
            RCLCPP_WARN(this->get_logger(), "SNR str: %s", buffer_); 
                
            micro_se = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

            snprintf(buffer, sizeof(buffer), "%.6f,%f", micro_se*1e-6, snr);
            __LOG_SNR::file() << buffer << std::endl;
        }

        // void shutdown_callback(void) {
void SnrLogger::shutdown_callback(void) {
            utility_serial_write(fd_, "sys set pindig GPIO10 0\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
            if (fd_ >= 0)
                utility_serial_close(fd_);
            #ifdef LOG_POSE
                __LOG_SNR::file().close();
                ROS_INFO_STREAM(LOG_SNR_FILE << " closed");
            #endif
        }
// };
// BsSnrPublisher::BsSnrPublisher(CommProtocol commprotocol_) : 
//     Node(NODE_CHW_BS_COMM), count_(0), count__(0), first_get(0), fd_(-1) {

//     string commprotocol_str = "802.11";

//     // BS_ADDR for the address and BS_PORT for the port. When setting up
//     // the address was 172.28.143.122 and the port was 80. Be aware, 
//     // it might change
//     if (commprotocol_ == CommProtocol::_80211) {
//         addr = string(PROTOCOL)    + "://" +
//                getenv(ENV_BS_ADDR) + ":"   +
//                getenv(ENV_BS_PORT) + string(DIR_ON_BS);

//         // subscriber to the image raw topic which sends images from the
//         // navigation camera
//         subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(CAMNAVRGB_TOPIC, 10, std::bind(&BsSnrPublisher::camnavrgb_callback, this, _1));
//         timer__ = this->create_wall_timer(
//             std::chrono::milliseconds(CHW_QUEUE_EMPTIER_RATE), std::bind(&BsSnrPublisher::queue_emptier_callback, this)
//         );
//         RCLCPP_INFO(this->get_logger(), "Companion HW --- base-station is set to 802.11");
//     } else {
//         commprotocol_str = "LoRa";
//         // navigation camera image transfer via LoRa is not implemented yet
//         RCLCPP_WARN(this->get_logger(), "Navigation images are not transfered on the channel companion HW --- base-station with LoRa (use 802.11 instead if needed)");
//     }

//     commprotocol__ = commprotocol_;
//     RCLCPP_INFO(this->get_logger(), "Companion HW --- base-station is set to %s", commprotocol_str.c_str());

//     publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>(COMM_FROM_BS_TOPIC, 10);
//     // publishes the data from the base-station
//     // use CHW_BS_COMM_RATE_HZ to personalize frequency in milliseconds
//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(1000/CHW_BS_SNR_RATE_HZ), std::bind(&BsSnrPublisher::snr_callback, this)
//     );
//     // the node publishes a message containing the two commands from
//     // base-station: offset on x and on y, from -100 to 100. Default
//     // value is 0, 0 for x, y
//     msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
//     msg_.layout.dim[0].size = 2;
//     msg_.layout.dim[0].stride = 1;
//     msg_.layout.dim[0].label = COMM_FROM_BS_TOPIC_LBL;

//     rclcpp::on_shutdown(std::bind(&BsSnrPublisher::shutdown_callback, this));
// }

// void BsSnrPublisher::snr_callback(void) {

//     static size_t _count      = 0;
//     char          buffer[100];
//     string cmd, output, output_;
//     msg_.data.clear();

//     //if (commprotocol__ == CommProtocol::_80211) {
// 	// communication channel between chw---bs uses 802.11
// //	if (count__ == 0) { // testing if all is okay the first time
// //	    output = utility_exec(("curl -Is " + addr + "/tele2_data | head -n 1").c_sstr()0;
// //	    // is only able to check the connection if the protocol is http
// //	    if (string(PROTOCOL) == "http" && output.find("200 OK") == std::string::npos) {
// //		RCLCPP_FATAL(this->get_logger(), "No connection to the base-station %s", addr.c_str());
// //		return;
// //	    }
// //	    RCLCPP_INFO(this->get_logger(), "Established connection with the base-station");
// //	}
// //				 //
// //	try {
// //	    output = utility_exec(("curl -s " + addr + "/tele2_data").c_str());
// //	} catch (...) {
// //	    RCLCPP_ERROR(this->get_logger(), "Error opening popen while retrieving data from base-station");
// //	    return;
// //	}
// //  } else { // communication channel between chw---bs uses LoRa
// //        if (count__ == 0) {
//             // LoRa needs to be initialized... Also testing if all is
//             // okay the first time with LoRa (similarly as with 802.11)
// //	    fd_ = utility_serial_open(DEF_PORT_READ);
// //	    utility_serial_write(fd_, "sys factoryRESET\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
// //	    std::this_thread::sleep_for(std::chrono::milliseconds(PAUSE_SYSRESET_RN2903));
// //	    output_ = utility_serial_read(fd_, "sys get ver\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
// //	    RCLCPP_INFO(this->get_logger(), "%s", output_.c_str());
// //	    utility_serial_write(fd_, "mac pause\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
// //	    utility_serial_write(fd_, "radio set pwr 10\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
// //          utility_serial_write(fd_, "radio set mod fsk\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
// //            output_ = utility_serial_read(fd_, "radio rx 0\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
// //            RCLCPP_INFO(this->get_logger(), "%s", output_.c_str());
// //            std::this_thread::sleep_for(std::chrono::milliseconds(PAUSE_FIRST_RX_RN2903));
// //
// //	    if (output_.find("ok") != std::string::npos) {
// //                RCLCPP_INFO(this->get_logger(), "Established connection with the base-station via LoRa");
// //            } else {
// //                RCLCPP_FATAL(this->get_logger(), "No connection to the base-station utilizing LoRa");
// //	        return;
// //	    }
// //	} try {
// //	    output_ = utility_serial_read(fd_, "radio rx 0\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
// //	    RCLCPP_INFO(this->get_logger(), "%s", output_.c_str());
// //	
// //	    if ((rx_pos = output_.find("radio_rx")) != std::string::npos) {
// //	         utility_serial_write(fd_, "sys set pindig GPIO10 1\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
// //	    } else {
// //	         RCLCPP_WARN(this->get_logger(), "Unable to retrieve data from the channel with LoRa this time");
// //	         return;
// //	    }
// //	
// //	    utility_serial_write(fd_, "sys set pindig GPIO10 0\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
// //	    output = "x:" + std::to_string((int)std::stoul(output_.substr(rx_pos+10,2), nullptr, 16) - 100) + " " +
// //	    "y:" + std::to_string((int)std::stoul(output_.substr(rx_pos+12,2), nullptr, 16) - 100);
// //	} catch (...) { 
// //	    RCLCPP_FATAL(this->get_logger(), "Cannot communicate with the device");
// //	    return;
// //	}
// //    }

//     if (_count++ == 0) {
// 	__LOG_SNR::file().open(LOG_SNR_FILE, std::ios::app);
// 	RCLCPP_INFO_STREAM(this->get_logger(), "logging enabled, check file " << LOG_SNR_FILE);
//         snprintf(buffer, sizeof(buffer), "time,success_rate");
//         __LOG_SNR::file() << buffer << std::endl;
//     }

//     output_ = utility_serial_read(fd_, "radio get snr\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
//     RCLCPP_WARN(this->get_logger(), "snr command output %s", output_.c_str());
//     try {
// 	snr = atof(output_.c_str());
//     } catch (...) {
//         RCLCPP_ERROR(this->get_logger(), "Unable to read SNR");
//     }

//     RCLCPP_WARN(this->get_logger(), "SNR: %f", snr); 
        
//     snr = -128.0;


//     __LOG_SNR::file() << buffer << std::endl;
// }

// void BsSnrPublisher::shutdown_callback(void) {
//     utility_serial_write(fd_, "sys set pindig GPIO10 0\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
//     if (fd_ >= 0)
//         utility_serial_close(fd_);
// #ifdef LOG_POSE
//     __LOG_SNR::file().close();
//     ROS_INFO_STREAM(LOG_SNR_FILE << " closed");
// #endif
// }
        
int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SnrLogger>());//(CommProtocol::LoRa));
    rclcpp::shutdown();
    return 0;
}


