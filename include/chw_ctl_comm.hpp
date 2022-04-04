
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
#define COMM_TO_CTL_TOPIC_LBL "linear_velocities"
#define ENV_CHW_CTL_DIR       "CHW_CTL_DIR"


namespace ytcg {
    class CtlCommPublisher : public rclcpp::Node {
    public:
        CtlCommPublisher(void);

    private:
        void topic_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
        std_msgs::msg::Float32MultiArray msg_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subscription_;
        size_t count_;
    };
}

#endif


