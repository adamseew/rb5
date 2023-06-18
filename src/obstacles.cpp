
//
// ██████╗ ██████╗ ███████╗
// ██╔══██╗██╔══██╗██╔════╝
// ██████╔╝██████╔╝███████╗
// ██╔══██╗██╔══██╗╚════██║
// ██║  ██║██████╔╝███████║
// ╚═╝  ╚═╝╚═════╝ ╚══════
//  RB5 Ground Robot ROS2 package, obstacles node source code
//
//  Supplementary material to the paper "A low-cost energy-efficient approach for long-term autonomous exploration": https://adamseewald.cc/short/rb52023
//
//  Copyright (c) Adam Seewald, GRAB Lab at Yale University, Department of Mechanical Engineering and Materials Science 
//  Distributed under CC BY-NC-SA licence, details: https://creativecommons.org/licenses/by-nc-sa/4.0/
//


#include "../include/chw_bs_comm.hpp"
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
using std::placeholders::_2;
using std::placeholders::_3;


Obstacles::Obstacles() : 
    Node(NODE_OBSTACLES), count__(0), x_(0), y_(0), ctl_count(0) {
    
    subscription_.subscribe(this, LONGEST_DISTANCE_POINT1_TOPIC);
    subscription__.subscribe(this, LONGEST_DISTANCE_POINT2_TOPIC);
    subscription____.subscribe(this, MIN_DISTANCE_Z_TOPIC);

    subscription___ = this->create_subscription<geometry_msgs::msg::Pose>(
        ORBSLAM_FRAMES_TOPIC, 1, std::bind(&Obstacles::pose_topic_callback, this, _1)
                                                                         );

    RCLCPP_INFO_STREAM(this->get_logger(), "subscribed to " << ORBSLAM_FRAMES_TOPIC);

    sync_ = std::make_shared<message_filters::Synchronizer<policy>>(policy(10), subscription_, subscription__, subscription____);
    sync_->registerCallback(std::bind(&Obstacles::points_topic_callback, this, _1, _2, _3));

    RCLCPP_INFO_STREAM(this->get_logger(), "subscribed to " << 
        LONGEST_DISTANCE_POINT1_TOPIC << ", " << 
        LONGEST_DISTANCE_POINT2_TOPIC << ", " <<
        MIN_DISTANCE_Z_TOPIC << " using approximate time");

    publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>(COMM_FROM_BS_TOPIC, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(CTL_SEND_RATE), std::bind(&Obstacles::timer_callback, this)
    );

    RCLCPP_INFO_STREAM(this->get_logger(), "created publisher for topic " << COMM_FROM_BS_TOPIC);
    RCLCPP_INFO_STREAM(this->get_logger(), "node " << NODE_OBSTACLES << " is now initialized");

    this->declare_parameter("turning_ratio_x", TURNING_RATIO_X);
    this->declare_parameter("turning_ratio_y", TURNING_RATIO_Y);
    this->declare_parameter("init_velocity", INIT_VELOCITY);
    this->declare_parameter("min_turning_rate", MIN_TURNING_RATE);
    this->declare_parameter("max_turning_rate", MAX_TURNING_RATE);
}

void Obstacles::points_topic_callback(const geometry_msgs::msg::Point::ConstPtr& msg, const geometry_msgs::msg::Point::ConstPtr& _msg, const std_msgs::msg::Float32::ConstPtr& __msg) {
    
    Point3D       ld_point1, ld_point2,
                  midpoint;
    double        distance, _distance, turning_ratio_x, turning_ratio_y;
    static double _turning_ratio_x, _turning_ratio_y;
    int           x, y, init_velocity, min_turning_rate, max_turning_rate;
    static int    _init_velocity, _min_turning_rate, _max_turning_rate;

    msg_.data.clear();

    init_velocity = this->get_parameter("init_velocity").get_parameter_value().get<int>();
    turning_ratio_x = this->get_parameter("turning_ratio_x").get_parameter_value().get<double>();
    turning_ratio_y = this->get_parameter("turning_ratio_y").get_parameter_value().get<double>();
    min_turning_rate = this->get_parameter("min_turning_rate").get_parameter_value().get<int>();
    max_turning_rate = this->get_parameter("max_turning_rate").get_parameter_value().get<int>();

    if (count__++ == 0) {
        RCLCPP_INFO(this->get_logger(), "first points callback");   
	_init_velocity = init_velocity;
        _turning_ratio_x = turning_ratio_x;
	_turning_ratio_y = turning_ratio_y;
	_min_turning_rate = min_turning_rate;
        _max_turning_rate = max_turning_rate;
    } else {
        if (init_velocity != _init_velocity) {
            RCLCPP_WARN_STREAM(this->get_logger(), "velocity changed to " << init_velocity);
	    _init_velocity = init_velocity;
        }
        if (turning_ratio_x != _turning_ratio_x) {
            RCLCPP_WARN_STREAM(this->get_logger(), "turning ratio on the x axis changed to " << turning_ratio_x);
            _turning_ratio_x = turning_ratio_x;
        }
        if (turning_ratio_y != _turning_ratio_y) {
            RCLCPP_WARN_STREAM(this->get_logger(), "turning ratio on the x axis changed to " << turning_ratio_y);
            _turning_ratio_y = turning_ratio_y;
        }
        if (min_turning_rate != _min_turning_rate) {
            RCLCPP_WARN_STREAM(this->get_logger(), "minimum turning rate changed to " << min_turning_rate);
            _min_turning_rate = min_turning_rate;
        }
        if (max_turning_rate != _max_turning_rate) {
            RCLCPP_WARN_STREAM(this->get_logger(), "maximum turning rate changed to " << max_turning_rate);
            _max_turning_rate = max_turning_rate;
        }
    }

    ld_point1 = msg;
    ld_point2 = _msg;
    distance = std::abs(ld_point1.x-ld_point2.x);
    midpoint = (ld_point1+ld_point2)/2;
    _distance = midpoint.x;
    midpoint = midpoint/sqrt(pow(midpoint.x, 2)+pow(midpoint.y, 2)+pow(midpoint.z, 2))*POINT_MAX_DISTANCE;
    RCLCPP_INFO(this->get_logger(), "detected midpoint is (%f, %f, %f)", midpoint.x, midpoint.y, midpoint.z);

    x = (-100/MAX_FOV_REALSENSE_X)*_distance*turning_ratio_x;
    y = init_velocity;
    if (x != 0) {
        y *= turning_ratio_y;
	if (std::abs(x) < min_turning_rate)
            x = min_turning_rate*x/std::abs(x);
	else if (std::abs(x) > max_turning_rate)
            x = max_turning_rate*x/std::abs(x);
    }

    if (__msg->data < MIN_DISTANCE_Z) {
        RCLCPP_WARN(this->get_logger(), "rocker-bogie is too close to the obstacle (%f m away)", __msg->data);
        x = 0;
        y = 0;
    }

    RCLCPP_INFO(this->get_logger(), "highest distance is: %f", distance);

    __mutex.lock();
    x_ = x;
    y_ = y;
    ctl_count = CTL_COUNT;
    RCLCPP_INFO(this->get_logger(), "direction is x: %d, y: %d", x_, y_); 
    __mutex.unlock();
} 

void Obstacles::timer_callback(void) {

    if (count__ == 0)
        return;

    __mutex.lock();	
    if (--ctl_count <= 0) {
        RCLCPP_WARN(this->get_logger(), "control action is too old, check if the points are still transmitted");
        __mutex.unlock();
        return;
    }

    msg_.data.push_back(x_);
    msg_.data.push_back(y_);

    RCLCPP_INFO(this->get_logger(), "Publishing command x: %d, y: %d", x_, y_);
    __mutex.unlock();
    publisher_->publish(msg_);
}

void Obstacles::pose_topic_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {

    static size_t count_ = 0;

    if (count_ == 0)
        RCLCPP_INFO(this->get_logger(), "first pose callback");

    _mutex.lock();
    _point = msg;
    _mutex.unlock();
    
    if (!(count_++ % 10)) // report pose every ten callbacks
    	RCLCPP_INFO(this->get_logger(), "pose callback called %d times, pose (%f, %f, %f)", count_, msg->position.x, msg->position.y, msg->position.z);
}

Obstacles::~Obstacles(void) { }


int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Obstacles>());
    rclcpp::shutdown();

    return 0;
}

