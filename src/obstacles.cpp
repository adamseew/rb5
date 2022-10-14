
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


Obstacles::Obstacles() : 
    Node(NODE_OBSTACLES) {
    
    subscription_.subscribe(this, LONGEST_DISTANCE_POINT1_TOPIC);
    subscription__.subscribe(this, LONGEST_DISTANCE_POINT2_TOPIC);

    subscription___ = this->create_subscription<geometry_msgs::msg::Pose>(
        ORBSLAM_FRAMES_TOPIC, 1, std::bind(&Obstacles::pose_topic_callback, this, _1)
                                                                         );

    RCLCPP_INFO_STREAM(this->get_logger(), "subscribed to " << ORBSLAM_FRAMES_TOPIC);

    sync_ = std::make_shared<message_filters::Synchronizer<policy>>(policy(10), subscription_, subscription__);
    sync_->registerCallback(std::bind(&Obstacles::points_topic_callback, this, _1, _2));

    RCLCPP_INFO_STREAM(this->get_logger(), "subscribed to both " << 
        LONGEST_DISTANCE_POINT1_TOPIC  << ", " << 
        LONGEST_DISTANCE_POINT2_TOPIC << " using approximate time");

    publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>(COMM_FROM_BS_TOPIC, 10);

    RCLCPP_INFO_STREAM(this->get_logger(), "created publisher for topic " << COMM_FROM_BS_TOPIC);
    RCLCPP_INFO_STREAM(this->get_logger(), "node " << NODE_OBSTACLES << " is now initialized");
}


void Obstacles::points_topic_callback(const geometry_msgs::msg::Point::ConstPtr& msg, const geometry_msgs::msg::Point::ConstPtr& _msg) {
    
    static size_t count_ = 0;
    Point3D       ld_point1, ld_point2,
                  waypoint,  waypoint_;
    int           x_, y_;

    msg_.data.clear();

    if (count_++ == 0)
        RCLCPP_INFO(this->get_logger(), "first points callback");   

    ld_point1 = msg;
    ld_point2 = _msg;
    waypoint = (ld_point1+ld_point2)/2;
    _mutex.lock();
    waypoint_ = waypoint-_point;
    _mutex.unlock();
    x_ = 100*(waypoint_.x);
    y_ = 100*(waypoint_.y);


    msg_.data.push_back(x_);
    msg_.data.push_back(y_);
    
    RCLCPP_INFO(this->get_logger(), "Publishing command x: %d, y: %d", x_, y_);
    publisher_->publish(msg_);

    RCLCPP_INFO(this->get_logger(), "next waypoint is (%f, %f, %f)", waypoint.x, waypoint.y, waypoint.z); 

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

