#include "rosbridge_server_cpp/base64.h"

namespace rosbridge_server_cpp {

static char valueToChar(uint8_t val) {
  if(val < 26)
    return 'A' + val;
  else if(val < 52)
    return 'a' + val - 26;
  else if(val < 62)
    return '0' + val - 52;
  else if(val == 62)
    return '+';
  else if(val == 63)
    return '/';
  else
    return '\n';
}

void base64Encode(const Buffer& buf, std::string* output) {
  output->resize( (buf.size() + 2) / 3  * 4 );
  size_t output_offset = 0;
  for(size_t byte_offset = 0; byte_offset < buf.size(); byte_offset += 3) {
    output->operator[](output_offset) = valueToChar((buf[byte_offset] & 0b11111100) >> 2);
    if(byte_offset + 1 < buf.size()) {
      output->operator[](output_offset + 1) = valueToChar( ((buf[byte_offset] & 0b00000011) << 4) |
						((buf[byte_offset + 1] & 0b11110000) >> 4) );
      if(byte_offset + 2 < buf.size()) {
	output->operator[](output_offset + 2) = valueToChar( ((buf[byte_offset + 1] & 0b00001111) << 2) |
						  ((buf[byte_offset + 2] & 0b11000000) >> 6) );
	output->operator[](output_offset + 3) = valueToChar(buf[byte_offset + 2] & 0b00111111);
      }
      else {
	output->operator[](output_offset + 2) = valueToChar((buf[byte_offset + 1] & 0b00001111) << 2);
	output->operator[](output_offset + 3) = '=';
      }
    }
    else {
      output->operator[](output_offset + 1) = valueToChar((buf[byte_offset] & 0b00000011) << 4);
      output->operator[](output_offset + 2) = '=';
      output->operator[](output_offset + 3) = '=';
    }
    output_offset += 4;
  }
}

}
