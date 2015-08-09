#include "rosbridge_server_cpp/rosbridge_protocol_handler.h"
#include "rosbridge_server_cpp/rosbridge_server.h"
#include "boost/date_time/posix_time/posix_time_types.hpp"

namespace rosbridge_server_cpp {

RosbridgeProtocolHandler::~RosbridgeProtocolHandler(){
}

RosbridgeProtocolHandlerBase::RosbridgeProtocolHandlerBase(roscpp_message_reflection::NodeHandle& nh,
							   boost::shared_ptr<RosbridgeTransport>& transport)
  : nh_(nh), transport_(transport), status_level_(ERROR), next_service_call_id_(0) {
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
void RosbridgeProtocolHandlerBase::advertise(const std::string& topic, const std::string& type, const std::string& id,
					     const PublishOptions& options) {
  if(publishers_.find(topic) != publishers_.end()) { // already advertised
    StatusMessageStream(this, WARNING, id) << topic << " is already advertised";
    // TODO handle all cases here (same type, etc)
  }
  else {
    roscpp_message_reflection::Publisher publisher = nh_.advertise(topic, type);
    if(publisher) {
      StatusMessageStream(this, INFO, id) << "Publishing: topic=" << topic << ", type=" << type;
      PublisherInfo info;
      info.publisher = publisher;
      info.options = options;
      publishers_[topic] = info;
    }
    else {
      StatusMessageStream(this, ERROR, id) << "Failed to advertise: topic=" << topic << ", type=" << type;
    }
  }
}

void RosbridgeProtocolHandlerBase::unadvertise(const std::string& topic, const std::string& id) {
  size_t num_removed = publishers_.erase(topic);
  if(num_removed == 0) {
    StatusMessageStream(this, ERROR, id) << topic << " is not advertised";
  }
  else {
    StatusMessageStream(this, INFO, id) << "Unadvertising: topic=" << topic;
  }
}

static void weak_onSubscribeCallback(boost::weak_ptr<RosbridgeProtocolHandlerBase> this_weak, const std::string& topic,
				     const SubscribeOptions& options,
				     const boost::shared_ptr<const roscpp_message_reflection::Message>& message) {
  boost::shared_ptr<RosbridgeProtocolHandlerBase> this_ = this_weak.lock();
  if(this_) {
    this_->onSubscribeCallback(topic, options.message_send, message);
  }
}

void RosbridgeProtocolHandlerBase::subscribe(const std::string& topic, const std::string& type, const std::string& id,
					     const SubscribeOptions& options) {
  if(subscribers_.find(topic) != subscribers_.end()) { // already subscribed
    StatusMessageStream(this, WARNING, id) << topic << " is already subscribed";
    // TODO handle all cases here (same type, etc)
  }
  else {
    boost::weak_ptr<RosbridgeProtocolHandlerBase> weak_this(keep_alive_this_);
    roscpp_message_reflection::Subscriber subscriber = nh_.subscribe(topic, type,
            boost::bind(weak_onSubscribeCallback, weak_this, topic, options, _1));
    if(subscriber) {
      StatusMessageStream(this, INFO, id) << "Subscribing: topic=" << topic << ", type=" << type;
      SubscriberInfo info;
      info.subscriber = subscriber;
      info.options = options;
      subscribers_[topic] = info;
    }
    else {
      StatusMessageStream(this, ERROR, id) << "Failed to subscribe: topic=" << topic << ", type=" << type;
    }
  }
}

void RosbridgeProtocolHandlerBase::unsubscribe(const std::string& topic, const std::string& id) {
  size_t num_removed = subscribers_.erase(topic);
  if(num_removed == 0) {
    StatusMessageStream(this, ERROR, id) << topic << " is not subscribed";
  }
  else {
    StatusMessageStream(this, INFO, id) << "Unsubscribing: topic=" << topic;
  }
}

static bool weak_onServiceServerCallback(boost::weak_ptr<RosbridgeProtocolHandlerBase> this_weak, const std::string& service,
					 const ServiceServerOptions& options,
					 const roscpp_message_reflection::Message& request,
					 roscpp_message_reflection::Message& response) {
  boost::shared_ptr<RosbridgeProtocolHandlerBase> this_ = this_weak.lock();
  if(this_) {
    return this_->onServiceServerCallback(service, options, request, response);
  }
  return false;
}

void RosbridgeProtocolHandlerBase::advertiseService(const std::string& service, const std::string& type, const std::string& id,
						    const ServiceServerOptions& options) {
  if(service_servers_.find(service) != service_servers_.end()) { // already advertised service
    StatusMessageStream(this, WARNING, id) << service << " service is already advertised";
    // TODO handle all cases here (same type, etc)
  }
  else {
    boost::weak_ptr<RosbridgeProtocolHandlerBase> weak_this(keep_alive_this_);
    roscpp_message_reflection::ServiceServer server = nh_.advertiseService(service, type,
            boost::bind(weak_onServiceServerCallback, weak_this, service, options, _1, _2));
    if(server) {
      StatusMessageStream(this, INFO, id) << "Advertising Service: service=" << service << ", type=" << type;
      ServiceServerInfo info;
      info.server = server;
      info.options = options;
      service_servers_[service] = info;
    }
    else {
      StatusMessageStream(this, ERROR, id) << "Failed to advertise service: service=" << service << ", type=" << type;
    }
  }
}

void RosbridgeProtocolHandlerBase::unadvertiseService(const std::string& service, const std::string& id) {
  size_t num_removed = service_servers_.erase(service);
  if(num_removed == 0) {
    StatusMessageStream(this, ERROR, id) << service << " is not advertised";
  }
  else {
    StatusMessageStream(this, INFO, id) << "Unadvertising service: service=" << service;
  }
}


bool RosbridgeProtocolHandlerBase::onServiceServerCallback(const std::string& service,
							   const ServiceServerOptions& options,
							   const roscpp_message_reflection::Message& request,
							   roscpp_message_reflection::Message& response) {
  class PendingServiceCallScope {
  public:
    PendingServiceCallScope(ServiceCallCollection& collection,
			    PendingServiceCall& pending_call)
      : id_(pending_call.id), collection_(collection) {
      collection_.insert(ServiceCallCollection::value_type(id_, pending_call));
    }
    ~PendingServiceCallScope() {
      collection_.erase(id_);
    }
  private:
    std::string id_;
    ServiceCallCollection& collection_;
  };

  ROS_ERROR_STREAM("Calling service");
  PendingServiceCall call(boost::lexical_cast<std::string>(next_service_call_id_++), response);
  {
    boost::unique_lock<boost::mutex> lock(service_server_call_mutex_);
    PendingServiceCallScope call_scope(pending_service_calls_, call);
    sendServiceServerRequest(service, call.id, options, request);
    boost::system_time timeout = boost::get_system_time() + boost::posix_time::seconds(5);
    while(!call.finished) {
      if(!service_server_call_cv_.timed_wait(lock, timeout)) {
	break; // timed out
      }
    }
  }

  ROS_ERROR_STREAM("Service result: finished=" << call.finished << ", result=" << call.result);
  if(!call.finished)
    return false;
  return call.result;
}



RosbridgeProtocolHandlerBase::PublishHandler::PublishHandler(RosbridgeProtocolHandlerBase* handler,
							     const std::string& topic, const std::string& id) {
  std::map<std::string, PublisherInfo>::iterator itr = handler->publishers_.find(topic);
  if(itr != handler->publishers_.end()) { // not advertised
    pub_ = itr->second.publisher;
    message_.reset(new roscpp_message_reflection::Message(pub_.getMessageType()));
  }
  else {
    StatusMessageStream(handler, ERROR, id) << topic << " is not advertised";
  }
}

RosbridgeProtocolHandlerBase::PublishHandler::~PublishHandler() {
  if(pub_) {
    // TODO handle publish special cases (header, etc)
    pub_.publish(*message_);
  }
}





RosbridgeProtocolHandlerBase::ServiceCallHandler::ServiceCallHandler(RosbridgeProtocolHandlerBase* handler,
								     const std::string& service, const std::string& type,
								     const std::string& id)
  : handler_(handler), service_(service), type_(type), id_(id) {
  client_ = handler->nh_.serviceClient(service, type);
  if(client_) {
    request_.reset(new roscpp_message_reflection::Message(client_.getRequestType()));
    response_.reset(new roscpp_message_reflection::Message(client_.getResponseType()));
  }
  else {
    StatusMessageStream(handler_, ERROR, id_) << "Could not create service client: " << service_;
  }
}

RosbridgeProtocolHandlerBase::ServiceCallHandler::~ServiceCallHandler() {
  if(client_) {
    result_ = client_.call(*request_, *response_);
    handler_->sendServiceResponse(service_, id_, result_, *response_);
  }
}





RosbridgeProtocolHandlerBase::ServiceResponseHandler::ServiceResponseHandler(RosbridgeProtocolHandlerBase* handler,
									     const std::string& service, const std::string& id)
  : handler_(handler), pending_call_(NULL), lock_(handler->service_server_call_mutex_) {
  ServiceCallCollection::iterator itr = handler->pending_service_calls_.find(id);
  if(itr != handler->pending_service_calls_.end()) {
    pending_call_ = &itr->second;
  }
  else {
    StatusMessageStream(handler_, ERROR, id) << service << " no pending call found for call id: " << id;
  }
}

RosbridgeProtocolHandlerBase::ServiceResponseHandler::~ServiceResponseHandler() {
  if(pending_call_) {
    ROS_ASSERT(pending_call_->finished);
    handler_->service_server_call_cv_.notify_all();
  }
}

void RosbridgeProtocolHandlerBase::ServiceResponseHandler::resolve(bool result) {
  pending_call_->result = result;
  pending_call_->finished = true;
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
