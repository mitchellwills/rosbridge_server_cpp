#include "rosbridge_server_cpp/rosbridge_protocol_handler.h"
#include "rosbridge_server_cpp/rosbridge_server.h"

namespace rosbridge_server_cpp {

RosbridgeProtocolHandler::~RosbridgeProtocolHandler(){
}

RosbridgeProtocolHandlerBase::RosbridgeProtocolHandlerBase(roscpp_message_reflection::NodeHandle& nh,
							   RosbridgeTransport *transport)
  : nh_(nh), transport_(transport) {}

RosbridgeProtocolHandlerBase::~RosbridgeProtocolHandlerBase(){
}

void RosbridgeProtocolHandlerBase::onClose() {
  close();
}

void RosbridgeProtocolHandlerBase::close() {
  transport_->close();
  transport_.reset();
  subscribers_.clear();
  publishers_.clear();
}

void RosbridgeProtocolHandlerBase::advertise(const std::string& topic, const std::string& type) {
  if(publishers_.find(topic) != publishers_.end()) { // already advertised
    StatusMessageStream(this, WARNING) << topic << " is already advertised";
    // TODO handle all cases here (same type, etc)
  }
  else {
    roscpp_message_reflection::Publisher publisher = nh_.advertise(topic, type);
    if(publisher) {
      StatusMessageStream(this, INFO) << "Publishing: topic=" << topic << ", type=" << type;
      publishers_[topic] = publisher;
    }
    else {
      StatusMessageStream(this, WARNING) << "Failed to advertise: topic=" << topic << ", type=" << type;
    }
  }
}

void RosbridgeProtocolHandlerBase::unadvertise(const std::string& topic) {
  size_t num_removed = publishers_.erase(topic);
  if(num_removed == 0) {
    StatusMessageStream(this, WARNING) << topic << " is not advertised";
  }
  else {
    StatusMessageStream(this, INFO) << "Unadvertising: topic=" << topic;
  }
}

roscpp_message_reflection::Publisher RosbridgeProtocolHandlerBase::getPublisher(const std::string& topic) {
  std::map<std::string, roscpp_message_reflection::Publisher>::iterator itr = publishers_.find(topic);
  if(itr == publishers_.end()) { // not advertised
    return roscpp_message_reflection::Publisher();
  }
  else {
    return itr->second;
  }
}

void RosbridgeProtocolHandlerBase::subscribe(const std::string& topic, const std::string& type) {
  if(subscribers_.find(topic) != subscribers_.end()) { // already subscribed
    StatusMessageStream(this, WARNING) << topic << " is already subscribed";
    // TODO handle all cases here (same type, etc)
  }
  else {
    roscpp_message_reflection::Subscriber subscriber = nh_.subscribe(topic, type,
            boost::bind(&RosbridgeProtocolHandlerBase::onSubscribeCallback, this, topic, _1));
    if(subscriber) {
      StatusMessageStream(this, INFO) << "Subscribing: topic=" << topic << ", type=" << type;
      subscribers_[topic] = subscriber;
    }
    else {
      StatusMessageStream(this, WARNING) << "Failed to subscribe: topic=" << topic << ", type=" << type;
    }
  }
}

void RosbridgeProtocolHandlerBase::unsubscribe(const std::string& topic) {
  size_t num_removed = subscribers_.erase(topic);
  if(num_removed == 0) {
    StatusMessageStream(this, WARNING) << topic << " is not subscribed";
  }
  else {
    StatusMessageStream(this, INFO) << "Unsubscribing: topic=" << topic;
  }
}

RosbridgeProtocolHandlerBase::StatusMessageStream::StatusMessageStream(RosbridgeProtocolHandlerBase* handler, StatusLevel level)
  : handler_(handler), level_(level) {}
RosbridgeProtocolHandlerBase::StatusMessageStream::~StatusMessageStream() {
  ROS_INFO_STREAM(levelToString(level_) << ": " << stream_.str());
  handler_->sendStatusMessage(level_, stream_.str());
}


}
