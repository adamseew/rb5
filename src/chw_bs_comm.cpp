
#include "../include/chw_bs_comm.hpp"
#include "../include/utility.hpp"

#include <boost/range/adaptors.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <boost/filesystem.hpp>

#include <time.h>

#include <functional>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <thread>
#include <cmath>

#define BOOST_RANGE_ENABLE_CONCEPT_ASSERT 0

using namespace ytcg;
using namespace std::chrono;

using std::placeholders::_1;
using std::string;

namespace bfs = boost::filesystem;
namespace ba = boost::adaptors;

BsCommPublisher::BsCommPublisher() : BsCommPublisher::BsCommPublisher(CommProtocol::_80211) { }

BsCommPublisher::BsCommPublisher(CommProtocol commprotocol_) : 
    Node(NODE_CHW_BS_COMM), count_(0), count__(0), first_get(0), fd_(-1) {

    string commprotocol_str = "802.11";

    // BS_ADDR for the address and BS_PORT for the port. When setting up
    // the address was 172.28.143.122 and the port was 80. Be aware, 
    // it might change
    if (commprotocol_ == CommProtocol::_80211) {
        addr = string(PROTOCOL)    + "://" +
               getenv(ENV_BS_ADDR) + ":"   +
               getenv(ENV_BS_PORT) + string(DIR_ON_BS);

        // subscriber to the image raw topic which sends images from the
        // navigation camera
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(CAMNAVRGB_TOPIC, 10, std::bind(&BsCommPublisher::camnavrgb_callback, this, _1));
        timer__ = this->create_wall_timer(
            std::chrono::milliseconds(CHW_QUEUE_EMPTIER_RATE), std::bind(&BsCommPublisher::queue_emptier_callback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Companion HW --- base-station is set to 802.11");
    } else {
        commprotocol_str = "LoRa";
        // navigation camera image transfer via LoRa is not implemented yet
        RCLCPP_WARN(this->get_logger(), "Navigation images are not transfered on the channel companion HW --- base-station with LoRa (use 802.11 instead if needed)");
    }
#ifdef SAVE_STATS
    timer___ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&BsCommPublisher::stats_callback, this)
    );
    per_xseconds = 0;
#endif
    commprotocol__ = commprotocol_;
    RCLCPP_INFO(this->get_logger(), "Companion HW --- base-station is set to %s", commprotocol_str.c_str());

    publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>(COMM_FROM_BS_TOPIC, 10);
    // publishes the data from the base-station
    // use CHW_BS_COMM_RATE to personalize frequency in milliseconds
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(CHW_BS_COMM_RATE), std::bind(&BsCommPublisher::timer_callback, this)
    );
    // the node publishes a message containing the two commands from
    // base-station: offset on x and on y, from -100 to 100. Default
    // value is 0, 0 for x, y
    msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg_.layout.dim[0].size = 2;
    msg_.layout.dim[0].stride = 1;
    msg_.layout.dim[0].label = COMM_FROM_BS_TOPIC_LBL;

    rclcpp::on_shutdown(std::bind(&BsCommPublisher::shutdown_callback, this));
}

void BsCommPublisher::camnavrgb_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    	
    cv_bridge::CvImagePtr cv_ptr;
    milliseconds ms = duration_cast<milliseconds>(
        system_clock::now().time_since_epoch()
    );
    string stored_path;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV library bridge exception: %s", e.what());
        return;
    }

    cv::imwrite(stored_path = "image_" + std::to_string(count_++) + "-" + std::to_string(ms.count()) + ".jpg", cv_ptr->image);

    RCLCPP_INFO(this->get_logger(), "image stored as %s", stored_path.c_str());
}

void BsCommPublisher::queue_emptier_callback(void) {
    
    string target_path(getenv(ENV_QUEUE_PATH));
    const boost::regex my_filter("image.*\.jpg");
    boost::smatch what;
    std::vector<string> images;

    for (auto &entry: boost::make_iterator_range(bfs::directory_iterator(target_path), {})
         | ba::filtered(static_cast<bool (*)(const bfs::path &)>(&bfs::is_regular_file))
         | ba::filtered([&](const bfs::path &path){ return boost::regex_match(path.filename().string(), what, my_filter); })
        )
            // images matching the pattern "image*.jpg".
            images.push_back(entry.path().filename().string());
	
    if (images.size() > 0) {
	
        RCLCPP_INFO(this->get_logger(), "The queue has %d items", images.size());		
        for (auto image : images) {

            std::string curl_command = "curl ";
	    curl_command += "-s -F \"data=@" + target_path + image + "\" ";
            curl_command += addr + "/fromagent.php";
            
            RCLCPP_INFO(this->get_logger(), "%s", curl_command.c_str());  
            if (utility_exec(curl_command.c_str()).find("good job agent") != std::string::npos) {
			
                RCLCPP_INFO(this->get_logger(), "%s transfer ack", image.c_str());			
                // deleting image that has been sent correctly
                boost::filesystem::remove(image);					
                RCLCPP_INFO(this->get_logger(), "Queue is being freed");
            } else
                RCLCPP_INFO(this->get_logger(), "Something went wrong with the conncection");

        }
    } else  
        RCLCPP_INFO(this->get_logger(), "Nothing to send this time");

    ++count_;
}

void BsCommPublisher::timer_callback(void) {

    int x_, y_, rx_pos;
    string cmd, output, output_;
    msg_.data.clear();

    if (commprotocol__ == CommProtocol::_80211) {
             // communication channel between chw---bs uses 802.11
        if (count__ == 0) { // testing if all is okay the first time
            output = utility_exec(("curl -Is " + addr + "/tele2_data | head -n 1").c_str());
             // is only able to check the connection if the protocol is http
            if (string(PROTOCOL) == "http" && output.find("200 OK") == std::string::npos) {
                RCLCPP_FATAL(this->get_logger(), "No connection to the base-station %s", addr.c_str());
                return;
            }
	        RCLCPP_INFO(this->get_logger(), "Established connection with the base-station");
        }

        try {
            output = utility_exec(("curl -s " + addr + "/tele2_data").c_str());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error opening popen while retrieving data from base-station");
	        return;
        }
    } else { // communication channel between chw---bs uses LoRa
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
        } try {

        output_ = utility_serial_read(fd_, "radio rx 0\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
        RCLCPP_INFO(this->get_logger(), "%s", output_.c_str());


        if ((rx_pos = output_.find("radio_rx")) != std::string::npos) {
            utility_serial_write(fd_, "sys set pindig GPIO10 1\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unable to retrieve data from the channel with LoRa this time");
            return;
        }

        utility_serial_write(fd_, "sys set pindig GPIO10 0\r\n", DEF_PORT_READ, DEF_BITRATE_57600);
        output = "x:" + std::to_string((int)std::stoul(output_.substr(rx_pos+10,2), nullptr, 16) - 100) + " " +
                 "y:" + std::to_string((int)std::stoul(output_.substr(rx_pos+12,2), nullptr, 16) - 100);

    } catch (...) { 
        RCLCPP_FATAL(this->get_logger(), "Cannot communicate with the device");
        return;
    }}
    auto from_bs = utility_split(output, ' ');
    if (count__ == 0) 
        RCLCPP_INFO(this->get_logger(), "Raw data from base-station %s are %s", addr.c_str(), (from_bs.at(0) + " " + from_bs.at(1)).c_str());

    per_xseconds++;

    try {
        x_ = std::__cxx11::stoi(utility_split(from_bs.at(0), ':').at(1));
        y_ = std::__cxx11::stoi(utility_split(from_bs.at(1), ':').at(1));
    } catch (...) {
        x_ = std::numeric_limits<int>::max();
    } if (std::abs(x_) > 100 || std::abs(y_) > 100) {
        
        // something is wrong with the value; probably an error on the
        // base-station side
        RCLCPP_ERROR(this->get_logger(), "Data from base-station are corrupted");
        return;
    }
    msg_.data.push_back(x_);
    msg_.data.push_back(y_);

    RCLCPP_INFO(this->get_logger(), "Publishing command from base-station, x: %d, y: %d", x_, y_);
    publisher_->publish(msg_);

    ++count__;
}

#ifdef SAVE_STATS
void BsCommPublisher::stats_callback(void) {

    double        sc_rate;
    static size_t _count      = 0;
    uint64_t      micro_se;
    char          buffer[100];

    if (_count++ == 0) {
	__LOG_COMM::file().open(LOG_COMM_FILE, std::ios::app);
	RCLCPP_INFO_STREAM(this->get_logger(), "logging enabled, check file " << LOG_COMM_FILE);
        snprintf(buffer, sizeof(buffer), "time,success_rate");
        __LOG_COMM::file() << buffer << std::endl;
    }

    sc_rate = per_xseconds/(1.0/(CHW_BS_COMM_RATE/1000.0));

    RCLCPP_WARN(this->get_logger(), "Success communication rate: %f", sc_rate);
    per_xseconds = 0;

    micro_se = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    snprintf(buffer, sizeof(buffer), "%.6f,%f", micro_se*1e-6, sc_rate);
    __LOG_COMM::file() << buffer << std::endl;
}
#endif

void BsCommPublisher::shutdown_callback(void) {
    utility_serial_write(fd_, "sys set pindig GPIO10 0\r\n", DEF_PORT_READ, DEF_BITRATE_57600, PAUSE_RN2903);
    if (fd_ >= 0)
        utility_serial_close(fd_);
#ifdef LOG_POSE
    __LOG_COMM::file().close();
    ROS_INFO_STREAM(LOG_COMM_FILE << " closed");
#endif
}
        
int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BsCommPublisher>(CommProtocol::LoRa));
    rclcpp::shutdown();
    return 0;
}


