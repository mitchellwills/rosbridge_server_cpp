#include "rosbridge_server_cpp/rosbridge_transport.h"

namespace rosbridge_server_cpp {

RosbridgeTransport::RosbridgeTransport() {}

void RosbridgeTransport::setMessageHandler(boost::shared_ptr<MessageHandler> handler) {
  message_handler_ = handler;
}

void RosbridgeTransport::dispatchOnMessage(const Buffer& msg) {
  boost::shared_ptr<MessageHandler> handler_lock = message_handler_.lock();
  if(handler_lock) {
    handler_lock->onMessage(msg);
  }
}

void RosbridgeTransport::dispatchOnClose() {
  boost::shared_ptr<MessageHandler> handler_lock = message_handler_.lock();
  if(handler_lock) {
    handler_lock->onClose();
  }
}

RosbridgeTransportServer::RosbridgeTransportServer() {}

void RosbridgeTransportServer::setClientHandler(ClientHandler *handler) {
  boost::unique_lock<boost::mutex> lock(mutex_);
  client_handler_ = handler;
}

class NoOpMessageHandler : public MessageHandler {
public:
  virtual void onMessage(const Buffer& msg) {}
  virtual void onClose() {}
};

void RosbridgeTransportServer::dispatchOnClient(boost::shared_ptr<RosbridgeTransport> transport) {
  boost::unique_lock<boost::mutex> lock(mutex_);
  if(client_handler_) {
    client_handler_->onClient(transport);
  }
}

}
