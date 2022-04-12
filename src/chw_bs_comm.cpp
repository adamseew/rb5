
#include "../include/chw_bs_comm.hpp"
#include "../include/utility.hpp"

#include <functional>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <cmath>

using namespace ytcg;

using std::string;


BsCommPublisher::BsCommPublisher() : 
    Node(NODE_CHW_BS_COMM), count_(0), first_get(0) {

    publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>(COMM_FROM_BS_TOPIC, 10);
    // publishes the data from the base station
    // use CHW_BS_COMM_RATE to personalize frequency in milliseconds
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(CHW_BS_COMM_RATE), std::bind(&BsCommPublisher::timer_callback, this)
    );
    // the node publishes a message containing the two commands from
    // base station: offset on x and on y, from -100 to 100. Default
    // value is 0, 0 for x, y
    msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg_.layout.dim[0].size = 2;
    msg_.layout.dim[0].stride = 1;
    msg_.layout.dim[0].label = COMM_FROM_BS_TOPIC_LBL;
}

void BsCommPublisher::timer_callback() {

    int x_, y_;
    string output;
    // build the address for the base station. Use environment variables
    // BS_ADDR for the address and BS_PORT for the port. When setting up
    // the address was 172.28.143.122 and the port was 80. Be aware, 
    // it might change
    string addr = string(PROTOCOL)    + "://" +
                  getenv(ENV_BS_ADDR) + ":"   +
                  getenv(ENV_BS_PORT) + string(DIR_ON_BS);
    msg_.data.clear();

    if (!first_get) {
        output = utility_exec(("curl -Is " + addr + " | head -n 1").c_str());
	// is is only able to check the connection if the protocol is http
        if (string(PROTOCOL) == "http" && output.find("200 OK") == std::string::npos) {
            RCLCPP_ERROR(this->get_logger(), "No connection to the base station %s", addr.c_str());
            return;
        }
	first_get = 1;
	RCLCPP_INFO(this->get_logger(), "Established connection with the base station");
    }

    try {
        output = utility_exec(("curl -s " + addr).c_str());
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Error opening popen while retrieving data from base station");
	return;
    }
    auto from_bs = utility_split(output, ' ');
    RCLCPP_INFO(this->get_logger(), "Raw data from base station %s are %s", addr.c_str(), output.c_str());
    try {
        x_ = std::__cxx11::stoi(utility_split(from_bs.at(0), ':').at(1));
        y_ = std::__cxx11::stoi(utility_split(from_bs.at(1), ':').at(1));
    } catch (...) {
        x_ = std::numeric_limits<int>::max();
    } if (std::abs(x_) > 100 || std::abs(y_) > 100) {
        
        // something is wrong with the value; probably an error on the
	// base station side
        RCLCPP_ERROR(this->get_logger(), "But they seem corrupted");
        return;
    }
    msg_.data.push_back(x_);
    msg_.data.push_back(y_);

    RCLCPP_INFO(this->get_logger(), "Publishing command from base station, x: %d, y: %d", x_, y_);
    publisher_->publish(msg_);
}
        
int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BsCommPublisher>());
    rclcpp::shutdown();
    return 0;
}


