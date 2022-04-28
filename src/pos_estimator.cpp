
#include "../include/pos_estimator.hpp"
#include "../include/chw_bs_comm.hpp"
#include "../include/utility.hpp"

#include <functional>
#include <cstdlib>
#include <chrono>
#include <cstdio>
#include <limits>
#include <cmath>

using namespace ytcg;

using std::placeholders::_1;


PosEstimator::PosEstimator() : 
    Node(NODE_POS_ESTIMATOR), cmd(2, std::numeric_limits<short int>::min()), pos(2, 0), orientation(90.0) {

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(POS_TOPIC, 10);
    publisher__ = this->create_publisher<std_msgs::msg::Float32>(ORI_TOPIC, 10);
    subscription_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(COMM_FROM_BS_TOPIC, 10, std::bind(&PosEstimator::topic_callback, this, _1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(POS_ESTIMATION_RATE), std::bind(&PosEstimator::timer_callback, this)
    );
    
    msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg_.layout.dim[0].size = 2;
    msg_.layout.dim[0].stride = 1;
    msg_.layout.dim[0].label = POS_TOPIC_LBL;
}

void PosEstimator::topic_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
   
    cmd_mutex.lock();
    cmd.at(0) = msg->data[0];
    cmd.at(1) = msg->data[1];
    cmd_mutex.unlock();
}

void PosEstimator::timer_callback(void) {
    msg_.data.clear();
    
    if (pos.at(0) >= 0 && pos.at(1) >= 0) {
	orientation = orientation+cmd.at(1)*MAX_ANG_VEL*POS_ESTIMATION_RATE/1000.0;
	msg__.data = orientation;

	pos.at(0) = pos.at(0)+cos(M_PI*orientation/180.0)*cmd.at(0)*MAX_VEL*POS_ESTIMATION_RATE/1000.0;
	pos.at(1) = pos.at(1)+sin(M_PI*orientation/180.0)*cmd.at(0)*MAX_VEL*POS_ESTIMATION_RATE/1000.0;
        msg_.data.push_back(pos.at(0));
        msg_.data.push_back(pos.at(1));
    }

    RCLCPP_INFO(this->get_logger(), "Estimated position is x: %d, y: %d, and orientation is %f", pos.at(0), pos.at(1), orientation);

    publisher_->publish(msg_);
    publisher__->publish(msg__);
}

int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosEstimator>());
    rclcpp::shutdown();
    return 0;
}


