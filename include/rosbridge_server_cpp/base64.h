#ifndef ROSBRIDGE_SERVER_CPP_BASE64_H
#define ROSBRIDGE_SERVER_CPP_BASE64_H

#include <stdlib.h>
#include <stdint.h>
#include <string>
#include "rosbridge_server_cpp/buffer.h"

namespace rosbridge_server_cpp {

void base64Encode(const Buffer& buf, std::string* output);

}

#endif
