
#include <string>
#include <vector>

#ifndef YTCG_UTILITY_HPP
#define YTCG_UTILITY_HPP

namespace ytcg {
    
    std::vector<std::string> utility_split(const std::string& source, char token);
    const std::string utility_trim(const std::string& source);
    const std::string utility_exec(const char* cmd);
}

#endif


