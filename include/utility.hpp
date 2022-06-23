
#include <string>
#include <vector>

#include <termios.h>

#ifndef YTCG_UTILITY_HPP
#define YTCG_UTILITY_HPP

#define DEF_BITRATE_9600  0000015
#define DEF_BITRATE_57600 0010001
#define DEF_PORT_WRITE    "/dev/ttyTHS2"
#define DEF_PORT_READ     "/dev/ttyUSB0"
#define READ_BUFFER_SIZE  256

namespace ytcg {
    
    std::vector<std::string> utility_split(const std::string& source, char token);
    const std::string utility_trim(const std::string& source);
    const std::string utility_exec(const char* cmd);

    void utility_set_interface_attribs(int fd, int speed, int parity);
    void utility_set_blocking(int fd, int should_block);
    void utility_serial_write(int fd_, const std::string& data, const std::string& portname, speed_t bitrate, size_t sleep, bool blocking);
    void utility_serial_write(int fd_, const std::string& data, const std::string& portname, speed_t bitrate, size_t sleep);
    void utility_serial_write(int fd_, const std::string& data, const std::string& portname, speed_t bitrate);
    void utility_serial_write(const std::string& data, const std::string& portname, speed_t bitrate);
    void utility_serial_write(const std::string& data);
    std::string utility_serial_read(int fd_, const std::string& data, const std::string& portname, speed_t bitrate, size_t sleep_, bool blocking);
    std::string utility_serial_read(int fd_, const std::string& data, const std::string& portname, speed_t bitrate, size_t sleep_);
    std::string utility_serial_read(int fd_, const std::string& data, const std::string& portname, speed_t bitrate);
    std::string utility_serial_read(const std::string& data, const std::string& portname, speed_t bitrate);
    std::string utility_serial_read(const std::string& data);
    int utility_serial_open(const std::string& portname);
    void utility_serial_close(int fd);
}

#endif

