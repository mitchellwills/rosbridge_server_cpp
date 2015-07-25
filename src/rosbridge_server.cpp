#include "rosbridge_server_cpp/rosbridge_server.h"

namespace rosbridge_server_cpp {

RosbridgeServer::RosbridgeServer(ros::NodeHandle& nh)
  : nh_(nh) {
}
RosbridgeServer::~RosbridgeServer() {
  transport_server_->setClientHandler(NULL);
  active_clients_.close();
  transport_server_.reset();
}

void RosbridgeServer::init(RosbridgeTransportServer *transport_server) {
  transport_server_.reset(transport_server);
  transport_server_->setClientHandler(this);
}

boost::shared_ptr<MessageHandler>  RosbridgeServer::onClient(RosbridgeTransport *transport) {
  return active_clients_.add(new JsonRosbridgeProtocolHandler(nh_, transport));
}

}
