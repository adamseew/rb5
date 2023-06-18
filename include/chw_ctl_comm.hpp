
//
// ██████╗ ██████╗ ███████╗
// ██╔══██╗██╔══██╗██╔════╝
// ██████╔╝██████╔╝███████╗
// ██╔══██╗██╔══██╗╚════██║
// ██║  ██║██████╔╝███████║
// ╚═╝  ╚═╝╚═════╝ ╚══════
//  RB5 Ground Robot ROS2 package, communication with the microcontroller node header
//
//  Supplementary material to the paper "A low-cost energy-efficient approach for long-term autonomous exploration": https://adamseewald.cc/short/rb52023
//
//  Copyright (c) Adam Seewald, GRAB Lab at Yale University, Department of Mechanical Engineering and Materials Science 
//  Distributed under CC BY-NC-SA licence, details: https://creativecommons.org/licenses/by-nc-sa/4.0/
//


#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sstream>

#ifndef YTCG_CHW_CTL_COMM_HPP
#define YTCG_CHW_CTL_COMM_HPP

#define NODE_CHW_CTL_COMM     "chw_ctl_node"
#define CHW_CTL_COMM_RATE     500
#define ENV_CHW_CTL_DIR       "CHW_CTL_DIR"

#define COMM_ETHERNET         100
#define COMM_SERIAL           200

namespace ytcg {
    enum struct CommType { 
        Ethernet = COMM_ETHERNET, 
        Serial =   COMM_SERIAL
    };

    class CtlComm : public rclcpp::Node {
    public:
        CtlComm(void);
	    CtlComm(CommType commtype_);

    private:
        void topic_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subscription_;
    	CommType commtype__;
    	std::stringstream data_;
    };
}

#endif


