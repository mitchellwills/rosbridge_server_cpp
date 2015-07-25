#ifndef ROSBRIDGE_SERVER_CPP_ROSBRIDGE_TRANSPORT_H
#define ROSBRIDGE_SERVER_CPP_ROSBRIDGE_TRANSPORT_H

#include "rosbridge_server_cpp/buffer.h"
#include <boost/shared_ptr.hpp>

namespace rosbridge_server_cpp {

class RosbridgeTransport;

class MessageHandler {
public:
  virtual void onMessage(const Buffer& msg) = 0;
  virtual void onClose() = 0;
};

class ClientHandler {
public:
  /**
   * Receiver takes ownership of transport
   */
  virtual boost::shared_ptr<MessageHandler> onClient(RosbridgeTransport* transport) = 0;
};

class RosbridgeTransport {
public:
  RosbridgeTransport();

  virtual void sendMessage(const Buffer& msg) = 0;
  virtual void close() = 0;
};

class RosbridgeTransportServer {
public:
  RosbridgeTransportServer();
  void setClientHandler(ClientHandler *handler);

protected:
  boost::shared_ptr<MessageHandler> dispatchOnClient(RosbridgeTransport* transport);
private:
  ClientHandler* client_handler_;
};

}

#endif
