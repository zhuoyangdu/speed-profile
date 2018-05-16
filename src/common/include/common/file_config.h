#ifndef COMMON_INCLUDE_FILE_CONFIG_H_
#define COMMON_INCLUDE_FILE_CONFIG_H_

#include <iostream>
#include <fstream>
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

template <typename MessageType>
bool GetProtoFromFile(const std::string &file_name, MessageType *message) {
    std::fstream input(file_name, std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cout << "Error: failed to open config file." << file_name << std::endl;
        return false;
    }
    if (!message->ParseFromIstream(&input)) {
        std::cout << "Error: failed to parse file " << file_name << std::endl;
        return false;
    }
    return true;
}

#endif // COMMON_INCLUDE_FILE_CONFIG_H_
