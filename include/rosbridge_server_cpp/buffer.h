#ifndef ROSBRIDGE_SERVER_CPP_BUFFER_H
#define ROSBRIDGE_SERVER_CPP_BUFFER_H

#include <stdint.h>
#include <string>

namespace rosbridge_server_cpp {

/**
 * A constant lightweight byte buffer wrapper
 */
class Buffer {
public:
  Buffer(const std::string& str)
    : buffer_(str.data()), size_(str.size()) {}

  Buffer(const char * buffer, size_t size)
    : buffer_(buffer), size_(size) {}

  const char& operator[](size_t i) const {
    return buffer_[i];
  }

  const char* data() const {
    return buffer_;
  }

  const char* begin() const {
    return buffer_;
  }

  const char* end() const {
    return buffer_ + size_;
  }

  size_t size() const {
    return size_;
  }

private:
  const char * const buffer_;
  const size_t size_;
};

}

#endif
