#ifndef ROSBRIDGE_SERVER_CPP_ROSBRIDGE_PROTOCOL_HANDLER_H
#define ROSBRIDGE_SERVER_CPP_ROSBRIDGE_PROTOCOL_HANDLER_H

#include "rosbridge_server_cpp/buffer.h"
#include "rosbridge_server_cpp/rosbridge_transport.h"
#include <boost/enable_shared_from_this.hpp>
#include <roscpp_message_reflection/node_handle.h>
#include <roscpp_message_reflection/message.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace rosbridge_server_cpp {

class RosbridgeClient;

class RosbridgeProtocolHandler : public MessageHandler, public boost::enable_shared_from_this<RosbridgeProtocolHandler> {
public:
  enum StatusLevel {
    INVALID_LEVEL = -1, NONE, ERROR, WARNING, INFO
  };
  static std::string levelToString(StatusLevel level) {
    if(level == ERROR) {
      return "error";
    }
    else if(level == WARNING) {
      return "warning";
    }
    else if(level == INFO) {
      return "info";
    }
    else if(level == NONE) {
      return "none";
    }
    else {
      return "";
    }
  }

  static StatusLevel stringToLevel(const std::string& level_str) {
    if(level_str == "error") {
      return ERROR;
    }
    else if(level_str == "warning") {
      return WARNING;
    }
    else if(level_str == "info") {
      return INFO;
    }
    else {
      return INVALID_LEVEL;
    }
  }


  virtual ~RosbridgeProtocolHandler();
  virtual void init() = 0;
  virtual void close() = 0;
};

struct MessageSendOptions {
  MessageSendOptions() : compression("none") {}
  std::string compression;
};

struct PublishOptions {
};

struct SubscribeOptions {
  MessageSendOptions message_send;
};

struct ServiceServerOptions {
};

class RosbridgeProtocolHandlerBase : public RosbridgeProtocolHandler {
public:
  RosbridgeProtocolHandlerBase(roscpp_message_reflection::NodeHandle& nh,
			       boost::shared_ptr<RosbridgeTransport>& transport);
  virtual ~RosbridgeProtocolHandlerBase();

  virtual void init();

  virtual void close();
  void onClose();

  void advertise(const std::string& topic, const std::string& type, const std::string& id, const PublishOptions& options);
  void unadvertise(const std::string& topic, const std::string& id);
  void subscribe(const std::string& topic, const std::string& type, const std::string& id, const SubscribeOptions& options);
  void unsubscribe(const std::string& topic, const std::string& id);
  void advertiseService(const std::string& service, const std::string& type, const std::string& id,
			const ServiceServerOptions& options);
  void unadvertiseService(const std::string& service, const std::string& id);
  bool onServiceServerCallback(const std::string& service, const ServiceServerOptions& options,
			       const roscpp_message_reflection::Message& request,
			       roscpp_message_reflection::Message& response);
  roscpp_message_reflection::ServiceClient getServiceClient(const std::string& service,
							    const std::string& type);
  void setStatusLevel(StatusLevel level, const std::string& id);

  virtual void onSubscribeCallback(const std::string& topic,
				   const MessageSendOptions& options,
				   const boost::shared_ptr<const roscpp_message_reflection::Message>& message) = 0;
  virtual void sendServiceServerRequest(const std::string& service, const std::string& id,
					const ServiceServerOptions& options,
					const roscpp_message_reflection::Message& request) = 0;


  roscpp_message_reflection::Publisher getPublisher(const std::string& topic);

protected:
  virtual void sendStatusMessage(StatusLevel level, const std::string& id, const std::string& msg) = 0;

  class StatusMessageStream {
  public:
    StatusMessageStream(RosbridgeProtocolHandlerBase* handler, StatusLevel level, const std::string& id);
    ~StatusMessageStream();

    template <class T>
    StatusMessageStream& operator<<(const T& value) {
      stream_ << value;
      return *this;
    }
  private:
    RosbridgeProtocolHandlerBase* handler_;
    StatusLevel level_;
    const std::string& id_;
    std::stringstream stream_;
  };

protected:
  roscpp_message_reflection::NodeHandle nh_;
  boost::shared_ptr<RosbridgeTransport> transport_;

private:
  StatusLevel status_level_;
  boost::shared_ptr<RosbridgeProtocolHandlerBase> keep_alive_this_;
  void messageCallback(const boost::shared_ptr<const roscpp_message_reflection::Message>& message);

private:
  struct PublisherInfo {
    roscpp_message_reflection::Publisher publisher;
    PublishOptions options;
  };

  struct SubscriberInfo {
    roscpp_message_reflection::Subscriber subscriber;
    SubscribeOptions options;
  };

  struct ServiceServerInfo {
    roscpp_message_reflection::ServiceServer server;
    ServiceServerOptions options;
  };

  std::map<std::string, PublisherInfo> publishers_;
  std::map<std::string, SubscriberInfo> subscribers_;
  std::map<std::string, ServiceServerInfo> service_servers_;

  struct PendingServiceCall {
    PendingServiceCall(const std::string& id, roscpp_message_reflection::Message& response)
      : id(id), finished(false), response(response) {}

    std::string id;
    bool finished;
    bool result;
    roscpp_message_reflection::Message& response;
  };
  typedef std::map<std::string, PendingServiceCall&> ServiceCallCollection;

  boost::mutex service_server_call_mutex_;
  boost::condition_variable service_server_call_cv_;
  uint32_t next_service_call_id_;
  ServiceCallCollection pending_service_calls_;

protected:
  class PendingServiceCallResolver {
  public:
    PendingServiceCallResolver(RosbridgeProtocolHandlerBase* handler,
			       const std::string& service, const std::string& id);
    ~PendingServiceCallResolver();

    bool isActive() { return pending_call_; }
    roscpp_message_reflection::Message& getResponseMessage() { return pending_call_->response; }
    void resolve(bool result);
  private:
    PendingServiceCall* pending_call_;
    RosbridgeProtocolHandlerBase* handler_;
    boost::unique_lock<boost::mutex> lock_;
  };

};

}

#endif
