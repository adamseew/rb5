
#include "../include/utility.hpp"

#include <sys/stat.h>
#include <signal.h>
#include <stdlib.h>

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


