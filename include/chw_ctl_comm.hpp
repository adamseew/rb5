
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef YTCG_CHW_CTL_COMM_HPP
#define YTCG_CHW_CTL_COMM_HPP

#define NODE_CHW_CTL_COMM     "chw_ctl_node"
#define CHW_CTL_COMM_RATE     500
#define COMM_TO_CTL_TOPIC     "ground_based/joints"
#define COMM_FROM_CTL_TOPIC   "ground_based/position"
#define COMM_TO_CTL_TOPIC_LBL "linear_velocities"
#define COMM_FROM_CTL_LBL     "absolute_position"
#define ENV_CHW_CTL_DIR       "CHW_CTL_DIR"

#define COMM_ETHERNET         100
#define COMM_SERIAL           200

namespace ytcg {
    enum struct CommType { 
        Ethernet = COMM_ETHERNET, 
        Serial =   COMM_SERIAL
    };

    class CtlCommPublisher : public rclcpp::Node {
    public:
        CtlCommPublisher(void);
	CtlCommPublisher(CommType commtype_);

    private:
        void topic_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
        std_msgs::msg::Float32MultiArray msg_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subscription_;
        size_t count_;
	CommType commtype__;
    };
}

#endif


