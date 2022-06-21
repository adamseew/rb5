
#include "../include/chw_ctl_comm.hpp"
#include "../include/chw_bs_comm.hpp"
#include "../include/utility.hpp"

#include <termios.h>

#include <functional>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

using namespace ytcg;

using std::placeholders::_1;

CtlComm::CtlComm() : CtlComm::CtlComm(CommType::Ethernet) { }
                                              // default communication channel set to ethernet

CtlComm::CtlComm(CommType commtype_) : 
    Node(NODE_CHW_CTL_COMM) {

    subscription_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(COMM_FROM_BS_TOPIC, 10, std::bind(&CtlComm::topic_callback, this, _1));
    commtype__ = commtype_;
}

void CtlComm::topic_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
    
    std::stringstream().swap(data_);
    data_ << msg->data[0]/100.0 << ":";
    data_ << msg->data[1]/100.0 << " ";
    
    if (commtype__ == CommType::Ethernet) {
        RCLCPP_INFO(this->get_logger(), "Transmitting data over ethernet to the controller");
        std::ofstream ofs(getenv(ENV_CHW_CTL_DIR), std::ofstream::trunc);
        ofs << data_.str();                   // writing the content to the file where it can
                                              // accessed by CHW_CTL communication (nginx)
        ofs.close();
    } else {
        RCLCPP_INFO(this->get_logger(), "Establishing serial connection with the controller");
        utility_serial_write(data_.str(), "/dev/ttyTHS2", B9600);
                                              // bitrate for the setial commmunication
    }

    RCLCPP_INFO(this->get_logger(), "Transmitted %s using %s connection", data_.str().c_str(), commtype__ == CommType::Ethernet ? "ethernet" : "serial");
}
        
int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CtlComm>(CommType::Serial));
    rclcpp::shutdown();
    return 0;
}


