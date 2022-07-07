
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

}


Obstacles::~Obstacles(void) { }


int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Obstacles>());
    rclcpp::shutdown();

    return 0;
}

