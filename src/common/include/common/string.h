#ifndef COMMON_INCLUDE_COMMON_STRING_H_
#define COMMON_INCLUDE_COMMON_STRING_H_

#include <iostream>
#include <memory>
#include <fstream>
#include <string>

namespace common {

class StringUtils {

 public:
    StringUtils(){}

    void SplitString(const std::string& s, const std::string& c,
                     std::vector<std::string>* v) {
        std::string::size_type pos1, pos2;
        pos2 = s.find(c);
        pos1 = 0;
        while (std::string::npos != pos2) {
            v->push_back(s.substr(pos1, pos2 - pos1));
            pos1 = pos2 + c.size();
            pos2 = s.find(c, pos1);
        }
        if (pos1 != s.length())
            v->push_back(s.substr(pos1));
    }

};

}

#endif // COMMON_INCLUDE_COMMON_STRING_H_
