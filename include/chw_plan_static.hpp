
//
// ██████╗ ██████╗ ███████╗
// ██╔══██╗██╔══██╗██╔════╝
// ██████╔╝██████╔╝███████╗
// ██╔══██╗██╔══██╗╚════██║
// ██║  ██║██████╔╝███████║
// ╚═╝  ╚═╝╚═════╝ ╚══════
//  RB5 Ground Robot ROS2 package, static planner node header
//
//  Supplementary material to the paper "A low-cost energy-efficient approach for long-term autonomous exploration": https://adamseewald.cc/short/rb52023
//
//  Copyright (c) Adam Seewald, GRAB Lab at Yale University, Department of Mechanical Engineering and Materials Science 
//  Distributed under CC BY-NC-SA licence, details: https://creativecommons.org/licenses/by-nc-sa/4.0/
//


#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"

#include <boost/numeric/ublas/vector.hpp>

#ifndef YTCG_CHW_PLAN_STATIC_HPP
#define YTCG_CHW_PLAN_STATIC_HPP

#define NODE_CHW_PLAN_STATIC "chw_plan_static_node"
#define EPSILON  2
#define EPSILON2 10
#define KE       0.2


namespace ytcg {
    class StaticPlan : public rclcpp::Node {
    public:
        StaticPlan(void);

    private:
        void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
	void orientation_topic_callback(const std_msgs::msg::Float32::SharedPtr msg);
        std_msgs::msg::Int8MultiArray msg_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription__;
        size_t count_;
	float orientation_;
    };

    class Function {                          // generic function handler, override
                                              // to implement a path function
    public:
        virtual ~Function(void) { }
        virtual double get_value(boost::numeric::ublas::vector<double> point) = 0;
	virtual boost::numeric::ublas::vector<double> get_gradient(boost::numeric::ublas::vector<double> point) = 0;
    };

    class Line : public Function {            // y = mx+c, with m,c passed via constructor
    public:
        Line(double m_, double c_);
        ~Line(void);
        virtual double get_value(boost::numeric::ublas::vector<double> point);
        virtual boost::numeric::ublas::vector<double> get_gradient(boost::numeric::ublas::vector<double> point);

    private:
        double m, c;
    };

    class Circle : public Function {          // (x-xc)^2+(y-yc)^2-r^2, with xc,yc,r passed via constructor
    public:
        Circle(double xc_, double yc_, double r_);
	~Circle(void);
	virtual double get_value(boost::numeric::ublas::vector<double> point);
	virtual boost::numeric::ublas::vector<double> get_gradient(boost::numeric::ublas::vector<double> point);
    
    private:
        double xc, yc, r;
    };

}

#endif


