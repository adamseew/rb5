
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "message_filters/subscriber.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#ifndef YTCG_OBSTACLES_HPP
#define YTCG_OBSTACLES_HPP

#define NODE_OBSTACLES                "obstacles"

#define ORBSLAM_FRAMES_TOPIC          "/orb_slam3_ros_wrapper/pose"
#define LONGEST_DISTANCE_POINT1_TOPIC "/pointcloud_depth_wrapper/ld_point1"
#define LONGEST_DISTANCE_POINT2_TOPIC "/pointcloud_depth_wrapper/ld_point2"

namespace ytcg {

    struct Point3D {
        double x, y, z;
        Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) { }
        Point3D() : x(.0), y(.0), z(.0) { }
        Point3D& operator=(const geometry_msgs::msg::Pose::SharedPtr msg) {
            x = msg->position.x;
            y = msg->position.y;
            z = msg->position.z;
	    return *this;
	}
	Point3D& operator=(const geometry_msgs::msg::Point::ConstPtr& msg) {
            x = msg->x;
            y = msg->y;
            z = msg->z;
	    return *this;
        }
	Point3D& operator-(const Point3D& _point) {
            x -= _point.x;
            y -= _point.y;
            z -= _point.z;
            return *this;
        }
        Point3D& operator+(const Point3D& _point) {
            x += _point.x;
            y += _point.y;
            z += _point.z;
            return *this;
        }
        Point3D& operator/(const int val) {
            x /= val;
            y /= val;
            z /= val;
            return *this;
        }
    };

    class Obstacles : public rclcpp::Node {
    public:
        Obstacles(void);
	~Obstacles(void);

    private:

        void pose_topic_callback(const geometry_msgs::msg::Pose::SharedPtr);
        void points_topic_callback(const geometry_msgs::msg::Point::ConstPtr&, const geometry_msgs::msg::Point::ConstPtr&);


	typedef message_filters::sync_policies::ApproximateTime<
                                      geometry_msgs::msg::Point,
                                      geometry_msgs::msg::Point>    policy;
	std::mutex                                                  _mutex;

        std::shared_ptr<message_filters::Synchronizer<policy>>      sync_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr   subscription___;
        message_filters::Subscriber<geometry_msgs::msg::Point>      subscription__;
        message_filters::Subscriber<geometry_msgs::msg::Point>      subscription_;
        Point3D                                                     _point;

        rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr publisher_;
	std_msgs::msg::Int8MultiArray                               msg_;
    };
}

#endif


