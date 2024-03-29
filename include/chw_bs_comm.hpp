
//
// ██████╗ ██████╗ ███████╗
// ██╔══██╗██╔══██╗██╔════╝
// ██████╔╝██████╔╝███████╗
// ██╔══██╗██╔══██╗╚════██║
// ██║  ██║██████╔╝███████║
// ╚═╝  ╚═╝╚═════╝ ╚══════
//  RB5 Ground Robot ROS2 package, communication with the base station node header
//
//  Supplementary material to the paper "A low-cost energy-efficient approach for long-term autonomous exploration": https://adamseewald.cc/short/rb52023
//
//  Copyright (c) Adam Seewald, GRAB Lab at Yale University, Department of Mechanical Engineering and Materials Science 
//  Distributed under CC BY-NC-SA licence, details: https://creativecommons.org/licenses/by-nc-sa/4.0/
//


#include "std_msgs/msg/multi_array_dimension.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

#include "rclcpp/rclcpp.hpp"

#include <fstream>
#include <string>

#ifndef YTCG_CHW_BS_COMM_HPP
#define YTCG_CHW_BS_COMM_HPP

#define NODE_CHW_BS_COMM       "chw_bs_node"
#define CHW_BS_COMM_RATE       100
#define CHW_QUEUE_EMPTIER_RATE 500
#define COMM_FROM_BS_TOPIC     "ground_based/cmd"
#define COMM_FROM_BS_TOPIC_LBL "commands"
#define ENV_BS_ADDR            "BS_ADDR"
#define ENV_BS_PORT            "BS_PORT"
#define ENV_QUEUE_PATH         "QUEUE_PATH"
#define PROTOCOL               "http"
                               // refers to the application layer protocol; do not confuse
                               // with the CommProtocol bellow (which refers to the physical
                               // layer protocol as well as, i.e., the device being used)
#define DIR_ON_BS              "/ground-based_master"
#define CAMNAVRGB_TOPIC        "image_raw/compressed"
#define COMM_80211             1000
#define COMM_LORA              2000
#define PAUSE_RN2903           10
#define PAUSE_SYSRESET_RN2903  8000
#define PAUSE_FIRST_RX_RN2903  1000
#define SAVE_STATS             1
                               // specifies whether to save statistics on the arrived and
			       // lost messages from the base station
#define LOG_COMM_FILE          "log_comm.dat"


namespace ytcg {
    enum struct CommProtocol { 
        _80211 = COMM_80211,   // communication happens via std. wifi protocols (e.g., 802.11g or
                               // the wide range 802.11ah comm. protocol)
        LoRa =   COMM_LORA     // communication relies on LoRa physical layer protocol
    };
#ifdef SAVE_STATS
    struct __LOG_COMM {
        static std::ofstream& file() { 
            static std::ofstream __file;
            return __file;
        }
    };
#endif

    class BsCommPublisher : public rclcpp::Node {
    public:
        BsCommPublisher(void);
        BsCommPublisher(CommProtocol);

    private:
        void timer_callback(void);
        void queue_emptier_callback(void);
        void shutdown_callback(void);
        void camnavrgb_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
#ifdef SAVE_STATS
        void stats_callback(void);
#endif
        std_msgs::msg::Int8MultiArray msg_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer__;
#ifdef SAVE_STATS
        rclcpp::TimerBase::SharedPtr timer___;
#endif
        rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr publisher_;
    	rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
        size_t count_;
        size_t count__;
#ifdef SAVE_STATS
	std::atomic<size_t> per_xseconds;
#endif
        size_t first_get;
        int fd_;
    	std::string addr;
        CommProtocol commprotocol__;
        std::thread rx_thread;
    };
}

#endif

