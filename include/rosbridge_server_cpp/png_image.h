#ifndef ROSBRIDGE_SERVER_CPP_PNG_IMAGE_H
#define ROSBRIDGE_SERVER_CPP_PNG_IMAGE_H

#include <stdlib.h>
#include <stdint.h>
#include <png.h>
#include <string>
#include <vector>
#include "rosbridge_server_cpp/buffer.h"

namespace rosbridge_server_cpp {

class PngImage {
public:
  PngImage();
  ~PngImage();

  void fromBuffer(size_t width, size_t height, const Buffer& buf, char padding = '\0');

  bool write(std::vector<char>* output_buffer);

  void freeImage();

private:
  size_t width_, height_;
  png_byte *data_;
  png_bytep *row_pointers_;
};

}

#endif
