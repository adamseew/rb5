
#include "../include/chw_ctl_comm.hpp"
#include "../include/utility.hpp"

#include <functional>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

using namespace ytcg;


CtlCommPublisher::CtlCommPublisher() : 
    Node(NODE_CHW_CTL_COMM), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(COMM_TO_CTL_TOPIC, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(CHW_CTL_COMM_RATE), std::bind(&CtlCommPublisher::timer_callback, this)
    );
    msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg_.layout.dim[0].size = 2;
    msg_.layout.dim[0].stride = 1;
    msg_.layout.dim[0].label = COMM_TO_CTL_TOPIC_LBL;
}

void CtlCommPublisher::timer_callback() {
    msg_.data.clear();
    msg_.data.push_back(10.0); // just trying with an arbitrary value for now
                               // it will have to contain the real values
                               // later on
    msg_.data.push_back(-10.0);
    

    std::ofstream ofs(getenv(ENV_CHW_CTL_DIR), std::ofstream::trunc);
    ofs << 10.0 << std::endl;
    ofs << -10.0;
    ofs.close();

    RCLCPP_INFO(this->get_logger(), "Publishing joints linear velocities, x: %f, y: %f", 10.0, -10.0);
    publisher_->publish(msg_);
}
        
int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CtlCommPublisher>());
    rclcpp::shutdown();
    return 0;
}


