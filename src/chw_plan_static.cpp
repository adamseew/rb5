
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

    // Function* f_ = new Line(1.0, 5.0);     // y = x+5
    Function* f_ = new Circle(10.0, 10.0, 5);
                                              // (x-10)^2+(y-10)^2-2.5^2
    float ke = KE;                            // gain determined empirically
    float dir, diff; 
    int x_, y_;
    matrix<double> E(2, 2, 0);
    E(0, 1) = -1.0;
    E(1, 0) = 1.0;

    vector<double> point(2), gradient, vf;
    point(0) = static_cast<double>(msg->data[0]);
    point(1) = static_cast<double>(msg->data[1]);
    gradient = f_->get_gradient(point);
    vf       = prod(E, gradient)-ke*f_->get_value(point)*gradient;
    dir = atan(vf(1)/vf(0));
    dir = dir*180/boost::math::constants::pi<double>();
                                              // transfor to dregrees to compare w/
                                              // orientation
    dir = fmod(dir+180, 360);                 // normalize dir [-pi,pi)
    dir += dir < 0 ? 180 : -180;
    
    RCLCPP_INFO(this->get_logger(), "dir: %f, orietnation: %f", dir, orientation_);

    diff = sqrt(pow(dir-orientation_, 2));

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

    RCLCPP_INFO(this->get_logger(), "Publishing the control, x: %d, y: %d; direction: %f", x_, y_, dir);

    delete f_;
}

Line::Line(double m_, double c_) {
    m = m_;
    c = c_;
}

Circle::Circle(double xc_, double yc_, double r_) {
    xc = xc_;
    yc = yc_;
    r = r_;
}

Line::~Line(void) { }

Circle::~Circle(void) { }

double Line::get_value(vector<double> point) {
    return point(1)-m*point(0)-c;
}

double Circle::get_value(vector<double> point) {
    return pow(point(0)-xc, 2)+pow(point(1)-yc, 2)-pow(r, 2);
}

vector<double> Line::get_gradient(vector<double> point) {
    vector<double> gradient(2);
    gradient(0) = -1.0*m;
    gradient(1) = 1.0;
    return gradient;
}

vector<double> Circle::get_gradient(vector<double> point) {
    vector<double> gradient(2);
    gradient(0) = 2*point(0)-2;
    gradient(1) = 2*point(1)-2;
    return gradient;
}

int main(int argc, char ** argv) {
   	
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticPlan>());
    rclcpp::shutdown();
    return 0;
}


