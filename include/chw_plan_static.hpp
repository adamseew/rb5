
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
#define EPSILON  5
#define EPSILON2 20



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

    class Function {
    public:
        virtual ~Function(void) { }
        virtual double get_value(boost::numeric::ublas::vector<double> point) = 0;
	virtual boost::numeric::ublas::vector<double> get_gradient(boost::numeric::ublas::vector<double> point) = 0;
    };

    class Line : public Function {
    public:
        Line(int m_, int c_);
        ~Line(void);
        virtual double get_value(boost::numeric::ublas::vector<double> point);
        virtual boost::numeric::ublas::vector<double> get_gradient(boost::numeric::ublas::vector<double> point);

    private:
        int m, c;
    };

}

#endif


