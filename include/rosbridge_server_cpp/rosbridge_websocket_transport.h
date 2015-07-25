#ifndef ROSBRIDGE_SERVER_CPP_ROSBRIDGE_WEBSOCKET_TRANSPORT_H
#define ROSBRIDGE_SERVER_CPP_ROSBRIDGE_WEBSOCKET_TRANSPORT_H

#include "rosbridge_server_cpp/rosbridge_transport.h"
#include <async_web_server_cpp/http_server.hpp>
#include <async_web_server_cpp/websocket_connection.hpp>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

namespace rosbridge_server_cpp {

class WebsocketMessageHandlerWrapper;

class WebsocketTransport : public RosbridgeTransport {
public:
  WebsocketTransport(async_web_server_cpp::WebsocketConnectionPtr websocket);
  ~WebsocketTransport();

  virtual void sendMessage(const Buffer& msg);
  virtual void close();

private:
  async_web_server_cpp::WebsocketConnectionPtr websocket_;

  friend class WebsocketMessageHandlerWrapper;
};

class WebsocketTransportServer : public RosbridgeTransportServer {
public:
  WebsocketTransportServer(const std::string &address, const std::string &port);
  ~WebsocketTransportServer();

private:
  async_web_server_cpp::WebsocketConnection::MessageHandler handleNewClient(const async_web_server_cpp::HttpRequest& request,
									    async_web_server_cpp::WebsocketConnectionPtr websocket);

private:
  boost::scoped_ptr<async_web_server_cpp::HttpServer> server_;
  async_web_server_cpp::HttpRequestHandlerGroup handler_group_;
};

}

#endif
