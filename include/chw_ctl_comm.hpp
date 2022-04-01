
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "rclcpp/rclcpp.hpp"

#define NODE_CHW_CTL_COMM     "chw_ctl_node"
#define COMM_TO_CTL_TOPIC     "ground_based/joints"
#define COMM_TO_CTL_TOPIC_LBL "linear_velocities"
#define ENV_CHW_CTL_DIR       "CHW_CTL_DIR"


namespace ytcg {
    class CtlCommPublisher : public rclcpp::Node {
    public:
        CtlCommPublisher(void);

    private:
        void timer_callback(void);
        std_msgs::msg::Float32MultiArray msg_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        size_t count_;
    };
}


