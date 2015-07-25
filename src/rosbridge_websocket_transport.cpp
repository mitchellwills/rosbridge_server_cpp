#include "rosbridge_server_cpp/rosbridge_websocket_transport.h"
#include <async_web_server_cpp/http_reply.hpp>
#include <async_web_server_cpp/websocket_request_handler.hpp>

namespace rosbridge_server_cpp {

WebsocketTransport::WebsocketTransport(async_web_server_cpp::WebsocketConnectionPtr websocket)
  : websocket_(websocket) {}
  WebsocketTransport::~WebsocketTransport() {
    printf("transport destruction\n");
  }

void WebsocketTransport::sendMessage(const Buffer& msg) {
  // save a copy in case close is called
  async_web_server_cpp::WebsocketConnectionPtr websocket = websocket_;
  if(!websocket)
    return;
  std::string msg_string(msg.data(), msg.size());

  websocket->sendTextMessage(msg_string);
  //std::cout << msg_string << std::endl;
}

void WebsocketTransport::close() {
  websocket_->close();
  websocket_.reset();
}


WebsocketTransportServer::WebsocketTransportServer(const std::string &address, const std::string &port)
  : handler_group_(async_web_server_cpp::HttpReply::stock_reply(async_web_server_cpp::HttpReply::not_found)) {
  handler_group_.addHandlerForPath("/", async_web_server_cpp::WebsocketHttpRequestHandler(
          boost::bind(&WebsocketTransportServer::handleNewClient, this, _1, _2)));

  server_.reset(new async_web_server_cpp::HttpServer(address, port, handler_group_, 1));
  server_->run();
}

WebsocketTransportServer::~WebsocketTransportServer() {
  server_->stop();
}

class WebsocketMessageHandlerWrapper {
public:
  WebsocketMessageHandlerWrapper(boost::shared_ptr<MessageHandler> handler) : handler_(handler) {}

  void operator()(const async_web_server_cpp::WebsocketMessage& message) {
    if(message.type == async_web_server_cpp::WebsocketMessage::type_text) {
      handler_->onMessage(message.content);
    }
    else if(message.type == async_web_server_cpp::WebsocketMessage::type_close) {
      handler_->onClose();
    }
    else {
      ROS_WARN_STREAM("Unexpected websocket message type: " << message.type << ": " << message.content);
    }
  }
private:
  boost::shared_ptr<MessageHandler> handler_;
};

async_web_server_cpp::WebsocketConnection::MessageHandler
WebsocketTransportServer::handleNewClient(const async_web_server_cpp::HttpRequest& request,
					  async_web_server_cpp::WebsocketConnectionPtr websocket) {
  return WebsocketMessageHandlerWrapper(dispatchOnClient(new WebsocketTransport(websocket)));
}

}
