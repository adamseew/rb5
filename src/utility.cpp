
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
#include <memory>
#include <string>
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

void ytcg::utility_serial_write(const string& data, const string& portname, speed_t bitrate) {
    
    int fd;

    fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
                                    // portname is, e.g., /dev/ttyS0 (J17 on TX2)
    if (fd < 0) {
        throw std::runtime_error("Error opening the device");
    }

    utility_set_interface_attribs(fd, bitrate, 0);
                                    // set speed to given bps, 8n1 (no par.)
    utility_set_blocking(fd, 0);    // set no blocking
    write(fd, data.c_str(), data.size());

    if (close(fd) < 0) {
        throw std::runtime_error("Error closing the device");
    }
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


