
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <mutex>

#ifndef YTCG_OBSTACLES_HPP
#define YTCG_OBSTACLES_HPP

#define NODE_OBSTACLES      "obstacle_node"

namespace ytcg {
    class Obstacles : public rclcpp::Node {
    public:
        Obstacles(void);
	~Obstacles(void);

    };
}

#endif


