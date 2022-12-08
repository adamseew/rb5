
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
    Node(NODE_OBSTACLES), count__(0), x_(0), y_(0), ctl_count(0) {
    
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
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(CTL_SEND_RATE), std::bind(&Obstacles::timer_callback, this)
    );

    RCLCPP_INFO_STREAM(this->get_logger(), "created publisher for topic " << COMM_FROM_BS_TOPIC);
    RCLCPP_INFO_STREAM(this->get_logger(), "node " << NODE_OBSTACLES << " is now initialized");
}

void Obstacles::points_topic_callback(const geometry_msgs::msg::Point::ConstPtr& msg, const geometry_msgs::msg::Point::ConstPtr& _msg) {
    
    Point3D       ld_point1, ld_point2,
                  midpoint;
    double        distance, _distance, z_distance;
    int           x, y;

    msg_.data.clear();

    if (count__++ == 0)
        RCLCPP_INFO(this->get_logger(), "first points callback");   

    ld_point1 = msg;
    ld_point2 = _msg;
    distance = std::abs(ld_point1.x-ld_point2.x);
    midpoint = (ld_point1+ld_point2)/2;
    _distance = midpoint.x;
    z_distance = std::min(ld_point1.z, ld_point2.z);
    RCLCPP_INFO(this->get_logger(), "debug 3 (%f, %f, %f)", midpoint.x, midpoint.y, midpoint.z);
    midpoint = midpoint/sqrt(pow(midpoint.x, 2)+pow(midpoint.y, 2)+pow(midpoint.z, 2))*POINT_MAX_DISTANCE;
    RCLCPP_INFO(this->get_logger(), "detected midpoint is (%f, %f, %f)", midpoint.x, midpoint.y, midpoint.z);

    // _mutex.lock();
    // midpoint = midpoint-_point;
    // _mutex.unlock();
    //

    RCLCPP_INFO(this->get_logger(), "debug 4 %f", _distance);
    
    if (_distance < 0) {
       x = INIT_VELOCITY;
       y = 0;
    } else if (_distance > 0) {
       x = -1*INIT_VELOCITY;
       y = 0;
    } else {
       x = 0;
       y = INIT_VELOCITY;
    }
    if (z_distance < MIDPOINT_MIN_DISTANCE_Z) {
        RCLCPP_WARN(this->get_logger(), "the gap is too narrow");
        x = 0;
        y = 0;
    }
    RCLCPP_INFO(this->get_logger(), "highest distance is: %f", distance);

    __mutex.lock();
    x_ = x;
    y_ = y;
    ctl_count = 10;
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

