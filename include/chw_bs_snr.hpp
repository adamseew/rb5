
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

#include "rclcpp/rclcpp.hpp"

#include <fstream>
#include <string>

#ifndef YTCG_CHW_BS_COMM_HPP
#define YTCG_CHW_BS_COMM_HPP

#define NODE_CHW_BS_SNR       "chw_bs_snr_node"
#define CHW_BS_SNR_RATE_MS     5000
#define CHW_QUEUE_EMPTIER_RATE 500
// #define SNR_TOPIC              "ground_based/snr"
// #define COMM_FROM_BS_TOPIC_LBL "commands"
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

                               // specifies whether to save statistics on the arrived and
			       // lost messages from the base station
#define LOG_SNR_FILE          "log_snr.dat"


namespace ytcg {
    enum struct CommProtocol { 
        _80211 = COMM_80211,   // communication happens via std. wifi protocols (e.g., 802.11g or
                               // the wide range 802.11ah comm. protocol)
        LoRa =   COMM_LORA     // communication relies on LoRa physical layer protocol
    };

    struct __LOG_SNR {
        static std::ofstream& file() { 
            static std::ofstream __file;
            return __file;
        }
    };


    class SnrLogger : public rclcpp::Node {
    public:
        SnrLogger(void);
        // SnrLogger(CommProtocol);

    private:
        void snr_callback(void);
        void shutdown_callback(void);
        
        // std_msgs::msg::Int8MultiArray msg_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer__;
        rclcpp::TimerBase::SharedPtr timer___;

        // rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr publisher_;
    	// rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
        size_t count_;
        size_t count__;
	
	    float snr;

        size_t first_get;
        int fd_;
    	std::string addr;
        CommProtocol commprotocol__;
        std::thread rx_thread;
    };
}

#endif

