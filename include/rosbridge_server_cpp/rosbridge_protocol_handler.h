#ifndef ROSBRIDGE_SERVER_CPP_ROSBRIDGE_PROTOCOL_HANDLER_H
#define ROSBRIDGE_SERVER_CPP_ROSBRIDGE_PROTOCOL_HANDLER_H

#include "rosbridge_server_cpp/buffer.h"
#include "rosbridge_server_cpp/rosbridge_transport.h"
#include <boost/enable_shared_from_this.hpp>
#include <roscpp_message_reflection/node_handle.h>
#include <roscpp_message_reflection/message.h>
#include <json/json.h>

namespace rosbridge_server_cpp {

class RosbridgeClient;

class RosbridgeProtocolHandler : public MessageHandler, public boost::enable_shared_from_this<RosbridgeProtocolHandler> {
public:
  enum StatusLevel {
    NONE, ERROR, WARNING, INFO
  };

  virtual ~RosbridgeProtocolHandler();
  virtual void close() = 0;
};

class RosbridgeProtocolHandlerBase : public RosbridgeProtocolHandler {
public:
  RosbridgeProtocolHandlerBase(roscpp_message_reflection::NodeHandle& nh,
			       RosbridgeTransport *transport);
  virtual ~RosbridgeProtocolHandlerBase();

  virtual void close();
  void onClose();

  void advertise(const std::string& topic, const std::string& type);
  void unadvertise(const std::string& topic);
  void subscribe(const std::string& topic, const std::string& type);
  void unsubscribe(const std::string& topic);

  virtual void onSubscribeCallback(const std::string& topic,
          const boost::shared_ptr<const roscpp_message_reflection::Message>& message) = 0;

  roscpp_message_reflection::Publisher getPublisher(const std::string& topic);

protected:
  class StatusMessageStream {
  public:
    StatusMessageStream(RosbridgeProtocolHandlerBase* handler, StatusLevel level);
    ~StatusMessageStream();

    template <class T>
    StatusMessageStream& operator<<(const T& value) {
      stream_ << value;
      return *this;
    }
  private:
    RosbridgeProtocolHandlerBase* handler_;
    StatusLevel level_;
    std::stringstream stream_;
  };
  virtual void sendStatusMessage(StatusLevel level, const std::string& msg) = 0;

protected:
  roscpp_message_reflection::NodeHandle nh_;
  boost::shared_ptr<RosbridgeTransport> transport_;

private:
  void messageCallback(const boost::shared_ptr<const roscpp_message_reflection::Message>& message);

private:
  std::map<std::string, roscpp_message_reflection::Publisher> publishers_;
  std::map<std::string, roscpp_message_reflection::Subscriber> subscribers_;
};

class JsonRosbridgeProtocolHandler : public RosbridgeProtocolHandlerBase {
public:
  JsonRosbridgeProtocolHandler(roscpp_message_reflection::NodeHandle& nh,
			       RosbridgeTransport *transport);
  virtual ~JsonRosbridgeProtocolHandler();

  virtual void onMessage(const Buffer& buf);
  virtual void onSubscribeCallback(const std::string& topic,
          const boost::shared_ptr<const roscpp_message_reflection::Message>& message);

  virtual void sendStatusMessage(StatusLevel level, const std::string& msg);

private:
  void onMessage(const Json::Value& msg);
  void sendMessage(const Json::Value& msg);
};

}

#endif
