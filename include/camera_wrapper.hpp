
//
// ██████╗ ██████╗ ███████╗
// ██╔══██╗██╔══██╗██╔════╝
// ██████╔╝██████╔╝███████╗
// ██╔══██╗██╔══██╗╚════██║
// ██║  ██║██████╔╝███████║
// ╚═╝  ╚═╝╚═════╝ ╚══════
//  RB5 Ground Robot ROS2 package, camera wrapper node header
//
//  Supplementary material to the paper "A low-cost energy-efficient approach for long-term autonomous exploration": https://adamseewald.cc/short/rb52023
//
//  Copyright (c) Adam Seewald, GRAB Lab at Yale University, Department of Mechanical Engineering and Materials Science 
//  Distributed under CC BY-NC-SA licence, details: https://creativecommons.org/licenses/by-nc-sa/4.0/
//


#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <mutex>

#ifndef YTCG_CAM_WRAPPER_HPP
#define YTCG_CAM_WRAPPER_HPP

#define NODE_CAM_WRAPPER "camera_wrapper_node"
#define CAM_WRAPPER_RATE 2000


namespace ytcg {
    class CameraWrapper : public rclcpp::Node {
    public:
        CameraWrapper(void);
	~CameraWrapper(void);

    private:
	void timer_callback(void);
        rclcpp::TimerBase::SharedPtr timer_;
	size_t count_;
    };
}

#endif


