#include "rosbridge_server_cpp/rosbridge_transport.h"

namespace rosbridge_server_cpp {

RosbridgeTransport::RosbridgeTransport() {}


RosbridgeTransportServer::RosbridgeTransportServer() {}

void RosbridgeTransportServer::setClientHandler(ClientHandler *handler) {
  client_handler_ = handler;
}

class NoOpMessageHandler : public MessageHandler {
public:
  virtual void onMessage(const Buffer& msg) {}
  virtual void onClose() {}
};

boost::shared_ptr<MessageHandler> RosbridgeTransportServer::dispatchOnClient(RosbridgeTransport* transport) {
  boost::shared_ptr<MessageHandler> message_handler;
  if(client_handler_) {
    message_handler = client_handler_->onClient(transport);
  }
  if(message_handler) {
    return message_handler;
  }
  else {
    delete transport;
    return boost::shared_ptr<MessageHandler>(new NoOpMessageHandler());
  }
}

}
