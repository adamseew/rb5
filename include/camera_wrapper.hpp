
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


