
//
// ██████╗ ██████╗ ███████╗
// ██╔══██╗██╔══██╗██╔════╝
// ██████╔╝██████╔╝███████╗
// ██╔══██╗██╔══██╗╚════██║
// ██║  ██║██████╔╝███████║
// ╚═╝  ╚═╝╚═════╝ ╚══════
//  RB5 Ground Robot ROS2 package, camera wrapper node source code
//
//  Supplementary material to the paper "A low-cost energy-efficient approach for long-term autonomous exploration": https://adamseewald.cc/short/rb52023
//
//  Copyright (c) Adam Seewald, GRAB Lab at Yale University, Department of Mechanical Engineering and Materials Science 
//  Distributed under CC BY-NC-SA licence, details: https://creativecommons.org/licenses/by-nc-sa/4.0/
//


#include "../include/camera_wrapper.hpp"
#include "../include/utility.hpp"

#include <functional>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <cstdio>
#include <limits>
#include <cmath>
#include <ctime>

using namespace ytcg;
using namespace std::chrono;

using std::placeholders::_1;


CameraWrapper::CameraWrapper() : 
    Node(NODE_CAM_WRAPPER), count_(0) {

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(CAM_WRAPPER_RATE), std::bind(&CameraWrapper::timer_callback, this)
    );    
}

CameraWrapper::~CameraWrapper(void) { }

void CameraWrapper::timer_callback(void) {
    
    std::string output, cam_command = "gphoto2 --capture-image-and-download --filename=camera_";

    milliseconds ms = duration_cast<milliseconds>(
        system_clock::now().time_since_epoch()
    );
    cam_command += std::to_string(count_++) + "-" + std::to_string(ms.count()) + ".jpg";
    output = utility_exec(cam_command.c_str());
	
    RCLCPP_INFO(this->get_logger(), "Took a picture with the sampling camera %s", output.c_str());

}

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraWrapper>());
    rclcpp::shutdown();

    return 0;
}

