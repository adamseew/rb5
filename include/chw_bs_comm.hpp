
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef YTCG_CHW_BS_COMM_HPP
#define YTCG_CHW_BS_COMM_HPP

#define NODE_CHW_BS_COMM       "chw_bs_node"
#define CHW_BS_COMM_RATE       500
#define COMM_FROM_BS_TOPIC     "ground_based/cmd"
#define COMM_FROM_BS_TOPIC_LBL "commands"
#define ENV_BS_ADDR            "BS_ADDR"
#define ENV_BS_PORT            "BS_PORT"
#define PROTOCOL               "http"
#define DIR_ON_BS              "/ground-based_master/tele2_data"


namespace ytcg {
    class BsCommPublisher : public rclcpp::Node {
    public:
        BsCommPublisher(void);

    private:
        void timer_callback(void);
        std_msgs::msg::Int8MultiArray msg_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr publisher_;
        size_t count_;
	size_t first_get;
    };
}

#endif


