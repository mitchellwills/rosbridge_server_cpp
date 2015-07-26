#include "rosbridge_server_cpp/png_image.h"

namespace rosbridge_server_cpp {

static void user_write_data(png_structp png_ptr, png_bytep data, png_size_t length) {
  std::vector<char> *output_buffer = (std::vector<char>*)png_get_io_ptr(png_ptr);
  size_t old_size = output_buffer->size();
  output_buffer->resize(old_size + length);
  memcpy(&output_buffer->front() + old_size, data, length);
}
static void user_flush_data(png_structp png_ptr) {
}


PngImage::PngImage() : width_(0), height_(0), data_(NULL), row_pointers_(NULL) {}

PngImage::~PngImage() {
  freeImage();
}

  void PngImage::fromBuffer(size_t width, size_t height, const Buffer& buf, char padding) {
  freeImage();
  width_ = width;
  height_ = height;
  size_t total_size = width * height * 3;
  data_ = (png_byte*)malloc(total_size);

  memcpy(data_, buf.begin(), buf.size());
  memset(data_ + buf.size(), padding, total_size - buf.size());

  row_pointers_ = (png_bytep*) malloc(sizeof(png_bytep) * height);
  for(size_t y = 0; y < height; y++) {
    size_t row_size = width * 3;
    row_pointers_[y] = data_ + y * row_size;
  }
}

bool PngImage::write(std::vector<char>* output_buffer) {
  output_buffer->clear();
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png_ptr) {
    return false;
  }

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
    return false;
  }

  png_set_write_fn(png_ptr, output_buffer, user_write_data, user_flush_data);

  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return false;
  }

  png_set_IHDR(png_ptr, info_ptr, width_, height_,
	       8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
	       PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

  png_write_info(png_ptr, info_ptr);

  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return false;
  }

  png_write_image(png_ptr, row_pointers_);

  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return false;
  }

  png_write_end(png_ptr, NULL);

  png_destroy_write_struct(&png_ptr, &info_ptr);

  return true;
}

void PngImage::freeImage() {
  if(row_pointers_) {
    free(row_pointers_);
    row_pointers_ = NULL;
  }
  if(data_) {
    free(data_);
    data_ = NULL;
  }
  width_ = 0;
  height_ = 0;
}

}
