
#include "../include/chw_plan_static.hpp"
#include "../include/pos_estimator.hpp"
#include "../include/chw_ctl_comm.hpp"
#include "../include/chw_bs_comm.hpp"
#include "../include/utility.hpp"

#include <boost/numeric/ublas/operation_blocked.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <functional>
#include <complex>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <cmath>


using namespace ytcg;
using namespace boost::numeric::ublas;
using namespace boost::assign;

using std::placeholders::_1;


StaticPlan::StaticPlan() : 
    Node(NODE_CHW_PLAN_STATIC), count_(0),
    orientation_(std::numeric_limits<float>::min()) {

    publisher_ = this->create_publisher<std_msgs::msg::Int8MultiArray>(COMM_FROM_BS_TOPIC, 10);
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(POS_TOPIC, 10, std::bind(&StaticPlan::topic_callback, this, _1));
    subscription__ = this->create_subscription<std_msgs::msg::Float32>(ORI_TOPIC, 10, std::bind(&StaticPlan::orientation_topic_callback, this, _1));
    
    msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg_.layout.dim[0].size = 2;
    msg_.layout.dim[0].stride = 1;
    msg_.layout.dim[0].label = COMM_FROM_BS_TOPIC_LBL;
}

void StaticPlan::orientation_topic_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    
    orientation_ = msg->data;
}

void StaticPlan::topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    
    if (orientation_ == std::numeric_limits<float>::min()) {
        RCLCPP_ERROR(this->get_logger(), "No data on orientation topic yet");
	return;
    }

    Function* line_ = new Line(1, 5);
    float ke = 0.05;
    float dir, diff; 
    int x_, y_;
    matrix<double> E(2, 2, 0);
    E(0, 1) = -1.0;
    E(1, 0) = 1.0;

    vector<double> point(2), gradient, vf;
    point(0) = static_cast<double>(msg->data[0]);
    point(1) = static_cast<double>(msg->data[1]);
    gradient = line_->get_gradient(point);
    vf       = prod(E, gradient)-ke*line_->get_value(point)*gradient;
    dir = atan(vf(1)/vf(0));

    dir = fmod(dir+180, 360);                 // normalize dir [-pi,pi)
    dir += dir < 0 ? 180 : -180;
    diff = std::norm(dir-orientation_);

    if (diff < EPSILON) {
        x_ = 0;
        y_ = 100;
    } else {
        if (dir-orientation_ < 0) {
            x_ = -100;
            y_ = 100;
        } else {
            x_ = 100;
            y_ = 100;
        }
        if (diff < EPSILON2) {
            y_ = 0;
	}
    }

    msg_.data.clear();
    msg_.data.push_back(x_);
    msg_.data.push_back(y_);
    publisher_->publish(msg_);

    RCLCPP_INFO(this->get_logger(), "The direction is %f degrees", dir*180/boost::math::constants::pi<double>());
    RCLCPP_INFO(this->get_logger(), "Publishing the control direction, x: %d, y: %d", x_, y_);

    delete line_;
}

Line::Line(int m_, int c_) {
    m = m_;
    c = c_;
}

Line::~Line(void) { }

double Line::get_value(vector<double> point) {
    return point(1)-m*point(0)-c;
}

vector<double> Line::get_gradient(vector<double> point) {
    vector<double> gradient(2);
    gradient(0) = -1.0*m;
    gradient(1) = 1.0;
    return gradient;
}

int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticPlan>());
    rclcpp::shutdown();
    return 0;
}


