#ifndef ROSBRIDGE_SERVER_CPP_ROSBRIDGE_SERVER_H
#define ROSBRIDGE_SERVER_CPP_ROSBRIDGE_SERVER_H

#include <roscpp_message_reflection/message_description_provider.h>
#include <roscpp_message_reflection/node_handle.h>
#include "rosbridge_server_cpp/rosbridge_transport.h"
#include "rosbridge_server_cpp/weak_collection.h"
#include "rosbridge_server_cpp/rosbridge_protocol_handler.h"

namespace rosbridge_server_cpp {

class RosbridgeClient;

class RosbridgeServer : public ClientHandler {
public:
  RosbridgeServer(ros::NodeHandle& nh);
  void init(RosbridgeTransportServer *transport_server);
  ~RosbridgeServer();

private:
  roscpp_message_reflection::NodeHandle nh_;
  boost::scoped_ptr<RosbridgeTransportServer> transport_server_;
  WeakCollection<RosbridgeProtocolHandler> active_clients_;

  boost::shared_ptr<MessageHandler> onClient(RosbridgeTransport *transport);
};

}

#endif
