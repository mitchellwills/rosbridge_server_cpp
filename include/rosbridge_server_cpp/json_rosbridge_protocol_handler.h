#ifndef ROSBRIDGE_SERVER_CPP_JSON_ROSBRIDGE_PROTOCOL_HANDLER_H
#define ROSBRIDGE_SERVER_CPP_JSON_ROSBRIDGE_PROTOCOL_HANDLER_H

#include "rosbridge_server_cpp/rosbridge_protocol_handler.h"
#include <json/json.h>

namespace rosbridge_server_cpp {

class JsonRosbridgeProtocolHandler : public RosbridgeProtocolHandlerBase {
public:
  JsonRosbridgeProtocolHandler(roscpp_message_reflection::NodeHandle& nh,
			       boost::shared_ptr<RosbridgeTransport>& transport);
  virtual ~JsonRosbridgeProtocolHandler();

  virtual void onMessage(const Buffer& buf);
  virtual void onSubscribeCallback(const std::string& topic,
          const boost::shared_ptr<const roscpp_message_reflection::Message>& message);

  virtual void sendStatusMessage(StatusLevel level, const std::string& id, const std::string& msg);

private:
  void onMessage(const Json::Value& msg);
  void sendMessage(const Json::Value& msg);
};

}

#endif
