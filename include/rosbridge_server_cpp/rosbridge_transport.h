#ifndef ROSBRIDGE_SERVER_CPP_ROSBRIDGE_TRANSPORT_H
#define ROSBRIDGE_SERVER_CPP_ROSBRIDGE_TRANSPORT_H

#include "rosbridge_server_cpp/buffer.h"
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

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
  virtual void onClient(boost::shared_ptr<RosbridgeTransport>& transport) = 0;
};

class RosbridgeTransport {
public:
  RosbridgeTransport();
  void setMessageHandler(boost::shared_ptr<MessageHandler> handler);

  virtual void sendMessage(const Buffer& msg) = 0;

protected:
  void dispatchOnMessage(const Buffer& msg);
  void dispatchOnClose();

private:
  boost::weak_ptr<MessageHandler> message_handler_;
};

class RosbridgeTransportServer {
public:
  RosbridgeTransportServer();
  void setClientHandler(ClientHandler *handler);
  virtual void stop() = 0;

protected:
  void dispatchOnClient(boost::shared_ptr<RosbridgeTransport> transport);
private:
  ClientHandler* client_handler_;
  boost::mutex mutex_;
};

}

#endif
