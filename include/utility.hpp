
#include <string>
#include <vector>

#ifndef YTCG_UTILITY_HPP
#define YTCG_UTILITY_HPP

namespace ytcg {
    
    std::vector<std::string> utility_split(const std::string& source, char token);
    const std::string utility_trim(const std::string& source);
    const std::string utility_exec(const char* cmd);

    void utility_set_interface_attribs(int fd, int speed, int parity);
    void utility_set_blocking(int fd, int should_block);
    void utility_serial_write(const std::string& data, const std::string& portname);
}

#endif


