
#include "../include/obstacles.hpp"
#include "../include/utility.hpp"

#include <functional>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <limits>
#include <cmath>

using namespace ytcg;
using namespace std::chrono;

using std::placeholders::_1;


Obstacles::Obstacles() : 
    Node(NODE_OBSTACLES) {
    
    subscription_ == this->create_subscription<sensor_msgs::msg::PointCloud2>("/rtabmap/cloud_obstacles", 1, std::bind(&Obstacles::topic_callback, this, _1));
}


void Obstacles::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    count_++;
    RCLCPP_INFO(this->get_logger(), "%d time called", count_);
}

Obstacles::~Obstacles(void) { }


int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Obstacles>());
    rclcpp::shutdown();

    return 0;
}

