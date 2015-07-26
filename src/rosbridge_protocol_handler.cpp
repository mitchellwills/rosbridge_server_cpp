#include "rosbridge_server_cpp/rosbridge_protocol_handler.h"
#include "rosbridge_server_cpp/rosbridge_server.h"

namespace rosbridge_server_cpp {

RosbridgeProtocolHandler::~RosbridgeProtocolHandler(){
}

RosbridgeProtocolHandlerBase::RosbridgeProtocolHandlerBase(roscpp_message_reflection::NodeHandle& nh,
							   boost::shared_ptr<RosbridgeTransport>& transport)
  : nh_(nh), transport_(transport), status_level_(ERROR) {
}

void RosbridgeProtocolHandlerBase::init() {
  keep_alive_this_ = boost::static_pointer_cast<RosbridgeProtocolHandlerBase>(shared_from_this());
  transport_->setMessageHandler(keep_alive_this_);
}

RosbridgeProtocolHandlerBase::~RosbridgeProtocolHandlerBase(){
  close();
}

void RosbridgeProtocolHandlerBase::onClose() {
  close();
}

void RosbridgeProtocolHandlerBase::close() {
  keep_alive_this_.reset(); // may result in destructor call
}
// TODO kep track of ids for publishers and subscribers
void RosbridgeProtocolHandlerBase::advertise(const std::string& topic, const std::string& type, const std::string& id) {
  if(publishers_.find(topic) != publishers_.end()) { // already advertised
    StatusMessageStream(this, WARNING, id) << topic << " is already advertised";
    // TODO handle all cases here (same type, etc)
  }
  else {
    roscpp_message_reflection::Publisher publisher = nh_.advertise(topic, type);
    if(publisher) {
      StatusMessageStream(this, INFO, id) << "Publishing: topic=" << topic << ", type=" << type;
      publishers_[topic] = publisher;
    }
    else {
      StatusMessageStream(this, WARNING, id) << "Failed to advertise: topic=" << topic << ", type=" << type;
    }
  }
}

void RosbridgeProtocolHandlerBase::unadvertise(const std::string& topic, const std::string& id) {
  size_t num_removed = publishers_.erase(topic);
  if(num_removed == 0) {
    StatusMessageStream(this, WARNING, id) << topic << " is not advertised";
  }
  else {
    StatusMessageStream(this, INFO, id) << "Unadvertising: topic=" << topic;
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

static void weak_onSubscribeCallback(boost::weak_ptr<RosbridgeProtocolHandlerBase> this_weak, const std::string& topic,
			 const boost::shared_ptr<const roscpp_message_reflection::Message>& message) {
  boost::shared_ptr<RosbridgeProtocolHandlerBase> this_ = this_weak.lock();
  if(this_) {
    this_->onSubscribeCallback(topic, message);
  }
}

void RosbridgeProtocolHandlerBase::subscribe(const std::string& topic, const std::string& type, const std::string& id) {
  if(subscribers_.find(topic) != subscribers_.end()) { // already subscribed
    StatusMessageStream(this, WARNING, id) << topic << " is already subscribed";
    // TODO handle all cases here (same type, etc)
  }
  else {
    boost::weak_ptr<RosbridgeProtocolHandlerBase> weak_this(keep_alive_this_);
    roscpp_message_reflection::Subscriber subscriber = nh_.subscribe(topic, type,
            boost::bind(weak_onSubscribeCallback, weak_this, topic, _1));
    if(subscriber) {
      StatusMessageStream(this, INFO, id) << "Subscribing: topic=" << topic << ", type=" << type;
      subscribers_[topic] = subscriber;
    }
    else {
      StatusMessageStream(this, WARNING, id) << "Failed to subscribe: topic=" << topic << ", type=" << type;
    }
  }
}

void RosbridgeProtocolHandlerBase::unsubscribe(const std::string& topic, const std::string& id) {
  size_t num_removed = subscribers_.erase(topic);
  if(num_removed == 0) {
    StatusMessageStream(this, WARNING, id) << topic << " is not subscribed";
  }
  else {
    StatusMessageStream(this, INFO, id) << "Unsubscribing: topic=" << topic;
  }
}

void RosbridgeProtocolHandlerBase::setStatusLevel(StatusLevel level, const std::string& id) {
  if(level != INVALID_LEVEL) {
    status_level_ = level;
  }
}

RosbridgeProtocolHandlerBase::StatusMessageStream::StatusMessageStream(RosbridgeProtocolHandlerBase* handler, StatusLevel level,
								       const std::string& id)
  : handler_(handler), level_(level), id_(id) {}
RosbridgeProtocolHandlerBase::StatusMessageStream::~StatusMessageStream() {
  if(level_ <= handler_->status_level_) {
    ROS_INFO_STREAM(levelToString(level_) << ": " << stream_.str());
    handler_->sendStatusMessage(level_, id_, stream_.str());
  }
}


}
