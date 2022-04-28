
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <vector>
#include <mutex>

#ifndef YTCG_POS_ESTIMATOR_HPP
#define YTCG_POS_ESTIMATOR_HPP

#define NODE_POS_ESTIMATOR  "post_estimator_node"
#define POS_ESTIMATION_RATE 100
#define ORI_TOPIC           "ground_based/orientation"
#define POS_TOPIC           "ground_based/pos"
#define POS_TOPIC_LBL       "position"
#define MAX_VEL             21.6
#define MAX_ANG_VEL         11


namespace ytcg {
    class PosEstimator : public rclcpp::Node {
    public:
        PosEstimator(void);

    private:
        void topic_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
	void timer_callback(void);
        std_msgs::msg::Float32MultiArray msg_;
	std_msgs::msg::Float32 msg__;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher__;
        rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subscription_;
	std::vector<short int> cmd;
	std::vector<int> pos;
        std::mutex cmd_mutex;
	float orientation;
    };
}

#endif


