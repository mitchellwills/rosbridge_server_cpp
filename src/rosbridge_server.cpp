#include "rosbridge_server_cpp/rosbridge_server.h"
#include "rosbridge_server_cpp/json_rosbridge_protocol_handler.h"

namespace rosbridge_server_cpp {

RosbridgeServer::RosbridgeServer(ros::NodeHandle& nh, RosbridgeTransportServer *transport_server)
  : nh_(nh) {
  transport_server_.reset(transport_server);
  transport_server_->setClientHandler(this);
}

RosbridgeServer::~RosbridgeServer() {
  transport_server_->stop();
  active_clients_.close();
}

void RosbridgeServer::onClient(boost::shared_ptr<RosbridgeTransport>& transport) {
  boost::shared_ptr<RosbridgeProtocolHandler> handler = active_clients_.add(new JsonRosbridgeProtocolHandler(nh_, transport));
  handler->init();
}

}
