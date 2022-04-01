
#include "../include/chw_bs_comm.hpp"
#include "../include/utility.hpp"

#include <functional>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

using namespace ytcg;
using namespace std::chrono_literals;

using std::string;


BsCommPublisher::BsCommPublisher() : 
    Node(NODE_CHW_BS_COMM), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>(COMM_FROM_BS_TOPIC, 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&BsCommPublisher::timer_callback, this)
    );
    msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg_.layout.dim[0].size = 2;
    msg_.layout.dim[0].stride = 1;
    msg_.layout.dim[0].label = COMM_FROM_BS_TOPIC_LBL;
}

void BsCommPublisher::timer_callback() {
    int x_, y_;
    msg_.data.clear();
    
    string addr = "http://172.28.143.122/ground-based_master/tele2_data";
    string output = utility_exec(("curl -s " + addr).c_str());
    auto from_bs = utility_split(output, ' ');
    RCLCPP_INFO(this->get_logger(), "Data from base station are %s", output.c_str());
    x_ = std::__cxx11::stoi(utility_split(from_bs.at(0), ':').at(1));
    y_ = std::__cxx11::stoi(utility_split(from_bs.at(1), ':').at(1));
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


