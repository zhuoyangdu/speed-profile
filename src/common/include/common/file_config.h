#ifndef COMMON_INCLUDE_FILE_CONFIG_H_
#define COMMON_INCLUDE_FILE_CONFIG_H_

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

namespace common {
template <typename MessageType>
bool SetProtoToASCIIFile(const MessageType &message, int file_descriptor) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::io::ZeroCopyOutputStream;
  if (file_descriptor < 0) {
    std::cout << "Invalid file descriptor." << std::endl;
    return false;
  }
  ZeroCopyOutputStream *output = new FileOutputStream(file_descriptor);
  bool success = TextFormat::Print(message, output);
  delete output;
  close(file_descriptor);
  return success;
}

/**
 * @brief Sets the content of the file specified by the file_name to be the
 *        ascii representation of the input protobuf.
 * @param message The proto to output to the specified file.
 * @param file_name The name of the target file to set the content.
 * @return If the action is successful.
 */
template <typename MessageType>
bool SetProtoToASCIIFile(const MessageType &message,
                         const std::string &file_name) {
  int fd = open(file_name.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
  if (fd < 0) {
    std::cout << "Unable to open file " << file_name << " to write." << std::endl;
    return false;
  }
  return SetProtoToASCIIFile(message, fd);
}

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


/**
 * @brief Parses the content of the file specified by the file_name as ascii
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
template <typename MessageType>
bool GetProtoFromASCIIFile(const std::string &file_name, MessageType *message) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    std::cout << "Failed to open file " << file_name << " in text mode." << std::endl;
    // Failed to open;
    return false;
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, message);
  if (!success) {
    std::cout << "Failed to parse file " << file_name << " as text proto." << std::endl;
  }
  delete input;
  close(file_descriptor);
  return success;
}

}
#endif // COMMON_INCLUDE_FILE_CONFIG_H_
