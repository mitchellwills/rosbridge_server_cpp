#include "rosbridge_server_cpp/rosbridge_websocket_transport.h"
#include <async_web_server_cpp/http_reply.hpp>
#include <async_web_server_cpp/websocket_request_handler.hpp>

namespace rosbridge_server_cpp {

WebsocketTransport::WebsocketTransport(async_web_server_cpp::WebsocketConnectionPtr websocket)
  : websocket_(websocket) {}

WebsocketTransport::~WebsocketTransport() {
}

void WebsocketTransport::sendMessage(const Buffer& msg) {
  std::string msg_string(msg.data(), msg.size());
  websocket_->sendTextMessage(msg_string);
}

WebsocketTransportServer::WebsocketTransportServer(const std::string &address, const std::string &port)
  : handler_group_(async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)) {
  handler_group_.addHandlerForPath("/", async_web_server_cpp::WebsocketHttpRequestHandler(
          boost::bind(&WebsocketTransportServer::handleNewClient, this, _1, _2)));

  server_.reset(new async_web_server_cpp::HttpServer(address, port, handler_group_, 1));
  server_->run();
}

WebsocketTransportServer::~WebsocketTransportServer() {
  stop();
}

void WebsocketTransportServer::stop() {
  server_->stop();
}

class WebsocketTransportWrapper {
public:
  WebsocketTransportWrapper(boost::shared_ptr<WebsocketTransport> transport) : transport_weak_(transport) {}

  void operator()(const async_web_server_cpp::WebsocketMessage& message) {
    boost::shared_ptr<WebsocketTransport> transport = transport_weak_.lock();
    if(message.type == async_web_server_cpp::WebsocketMessage::type_text) {
      transport->dispatchOnMessage(message.content);
    }
    else if(message.type == async_web_server_cpp::WebsocketMessage::type_close) {
      transport->dispatchOnClose();
    }
    else {
      ROS_WARN_STREAM("Unexpected websocket message type: " << message.type << ": " << message.content);
    }
  }
private:
  boost::weak_ptr<WebsocketTransport> transport_weak_;
};

async_web_server_cpp::WebsocketConnection::MessageHandler
WebsocketTransportServer::handleNewClient(const async_web_server_cpp::HttpRequest& request,
					  async_web_server_cpp::WebsocketConnectionPtr websocket) {
  boost::shared_ptr<WebsocketTransport> transport(new WebsocketTransport(websocket));
  dispatchOnClient(transport);
  return WebsocketTransportWrapper(transport);
}

}
