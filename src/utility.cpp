
#include "../include/utility.hpp"

#include <sys/stat.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h> 

#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <array>

using namespace ytcg;

using std::vector;
using std::string;


void ytcg::utility_set_interface_attribs(int fd, int speed, int parity) {
    
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) 
        throw std::runtime_error("Error while calling tcgetattr");
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
                                    // 8-bit chars; disable IGNBRK 
                                    // for mismatched speed tests; 
                                    // otherwise receive break as 
                                    // \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); 
                                    // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls;
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      
                                    // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0) 
        throw std::runtime_error("Error while calling tcsetattr");
}

void ytcg::utility_set_blocking(int fd, int should_block) {
        
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
        throw std::runtime_error("Error while calling tggetattr");

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        throw std::runtime_error("Error while setting term attributes");
}

void ytcg::utility_serial_write(const string& data) {
    utility_serial_write(data, DEF_PORT_WRITE, DEF_BITRATE_9600);
}

void ytcg::utility_serial_write(const string& data, const string& portname, speed_t bitrate) {
    utility_serial_write(-1, data, portname, bitrate);
}

void ytcg::utility_serial_write(int fd_, const string& data, const string& portname, speed_t bitrate) {
    utility_serial_write(fd_, data, portname, bitrate, 0);
}

void ytcg::utility_serial_write(int fd_, const string& data, const string& portname, speed_t bitrate, size_t sleep_) {
    
    int fd = fd_;
    bool keep_open = true;
    if (fd < 0) {
        fd = utility_serial_open(portname);
        bool keep_open = false;
    }

    utility_set_interface_attribs(fd, bitrate, 0);
                                    // set speed to given bps, 8n1 (no par.)
    utility_set_blocking(fd, 0);    // set no blocking
    write(fd, data.c_str(), data.size());

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_));

    if (!keep_open) 
        utility_serial_close(fd);
}

string ytcg::utility_serial_read(const string& data, const string& portname, speed_t bitrate) {
    
    return utility_serial_read(data, portname, bitrate);
}

string ytcg::utility_serial_read(const string& data) {
    
    return utility_serial_read(-1, data, DEF_PORT_READ, DEF_BITRATE_57600);
}

string ytcg::utility_serial_read(int fd_, const string& data, const string& portname, speed_t bitrate) {
    return utility_serial_read(fd_, data, portname, bitrate, 0);
}

string ytcg::utility_serial_read(int fd_, const string& data, const string& portname, speed_t bitrate, size_t sleep_) {
    
    int size_bytes, fd = fd_;
    bool keep_open = true;
    char read_buf[READ_BUFFER_SIZE];
    if (fd < 0) {
        fd = utility_serial_open(portname);
        bool keep_open = false;
    }

    utility_serial_write(fd, data, portname, bitrate);
    size_bytes = read(fd, &read_buf, sizeof(read_buf));

    if (size_bytes < 0)
        throw std::runtime_error("Error reading the device");

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_));

    if (!keep_open) 
        utility_serial_close(fd);

    return string(read_buf);
}

void ytcg::utility_serial_close(int fd) {
    if (close(fd) < 0)
        throw std::runtime_error("Error closing the device");
}

int ytcg::utility_serial_open(const string& portname) {
    int fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
                                    // portname is, e.g., /dev/ttyS0 (J17 on TX2)
    if (fd < 0)
        throw std::runtime_error("Error opening the device");
    
    return fd;
}

vector<string> ytcg::utility_split(const string& source, char token) {

    size_t          index;
    vector<string>  _source;
    string          __source(source);

    while (__source.size()) {

        index = __source.find(token);

        if (index != string::npos) {
            _source.push_back(__source.substr(0, index));
            __source = __source.substr(index + 1);
            if(__source.size() == 0)
                _source.push_back(__source);
        } else {
            _source.push_back(__source);
            __source = "";
        }
    }

    return _source;
}

const string ytcg::utility_trim(const string& source) {

    auto start = source.begin();
    while (start != source.end() && std::isspace(*start)) {
        start++;
    }

    auto end = source.end();
    do {
        end--;
    } while (std::distance(start, end) > 0 && std::isspace(*end));

    return string(start, end + 1);
}

const string ytcg::utility_exec(const char* cmd) {
    
    std::array<char, 128> buffer;
    string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);

    if (!pipe)
        throw std::runtime_error("Popen failed");

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        result += buffer.data();
    return result;
}


    