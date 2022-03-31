
#include <functional>
#include <fstream>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "rclcpp/rclcpp.hpp"

#define LOCAL_DATA_DIR "/var/www/chw-ctl/main"
#define COMM_STR       "linear_velocities"
#define NODE           "chw_ctl_comm"


using namespace std::chrono_literals;

class CtlCommPublisher : public rclcpp::Node {
    public:
        CtlCommPublisher() : Node(NODE), 
                             count_(0) {
            publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ground_based/joints", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&CtlCommPublisher::timer_callback, this)
            );
	    msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	    msg_.layout.dim[0].size = 2;
	    msg_.layout.dim[0].stride = 1;
	    msg_.layout.dim[0].label = COMM_STR;
        }
    private:
        void timer_callback() {
	    msg_.data.clear();
	    msg_.data.push_back(10.0); // just trying with an arbitrary value for now
	                               // it will have to contain the real values
				       // later on
	    msg_.data.push_back(-10.0);
            
            std::ofstream ofs(LOCAL_DATA_DIR, std::ofstream::trunc);
	    ofs << 10.0 << std::endl;
	    ofs << -10.0;
	    ofs.close();

	    RCLCPP_INFO(this->get_logger(), "Publishing joints linear velocities, x: %f, y: %f", 10.0, -10.0);
	    publisher_->publish(msg_);
        }
        
	std_msgs::msg::Float32MultiArray msg_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CtlCommPublisher>());
    rclcpp::shutdown();
    return 0;
}


