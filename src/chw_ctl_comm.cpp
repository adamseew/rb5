
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

CtlCommPublisher::CtlCommPublisher() : CtlCommPublisher::CtlCommPublisher(CommType::Ethernet) { }

CtlCommPublisher::CtlCommPublisher(CommType commtype_) : 
    Node(NODE_CHW_CTL_COMM), count_(0) {

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(COMM_TO_CTL_TOPIC, 10);
                                             // subscribes to the topic communicating the data
                                             // from the base station. Each time there are any
                                             // data, in sends them to the controller
    subscription_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(COMM_FROM_BS_TOPIC, 10, std::bind(&CtlCommPublisher::topic_callback, this, _1));
    //
    msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg_.layout.dim[0].size = 2;
    msg_.layout.dim[0].stride = 1;
    msg_.layout.dim[0].label = COMM_TO_CTL_TOPIC_LBL;
    commtype__ = commtype_;
}

void CtlCommPublisher::topic_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
    
    msg_.data.clear();
    msg_.data.push_back(msg->data[0]/100.0); // just trying with an arbitrary mapping for now
                                             // it will have to contain the real values
                                             // later on
    msg_.data.push_back(msg->data[1]/100.0);
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
    }

    RCLCPP_INFO(this->get_logger(), "Transmitted %susing %s connection", data_.str().c_str(), commtype__ == CommType::Ethernet ? "ethernet" : "serial");
    publisher_->publish(msg_);
}
        
int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CtlCommPublisher>(CommType::Serial));
    rclcpp::shutdown();
    return 0;
}


