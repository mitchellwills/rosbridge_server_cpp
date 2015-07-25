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

static std::string levelToString(RosbridgeProtocolHandler::StatusLevel level) {
  if(level == RosbridgeProtocolHandler::ERROR) {
    return "error";
  }
  else if(level == RosbridgeProtocolHandler::WARNING) {
    return "warning";
  }
  else if(level == RosbridgeProtocolHandler::INFO) {
    return "info";
  }
  else if(level == RosbridgeProtocolHandler::NONE) {
    return "none";
  }
  else {
    return "";
  }
}

RosbridgeProtocolHandlerBase::StatusMessageStream::StatusMessageStream(RosbridgeProtocolHandlerBase* handler, StatusLevel level)
  : handler_(handler), level_(level) {}
RosbridgeProtocolHandlerBase::StatusMessageStream::~StatusMessageStream() {
  ROS_INFO_STREAM(levelToString(level_) << ": " << stream_.str());
  handler_->sendStatusMessage(level_, stream_.str());
}


class JsonValueAssignmentVisitor : public boost::static_visitor<> {
public:
  JsonValueAssignmentVisitor(Json::Value& json_value) : json_value_(json_value) {}

  template <typename T>
  void operator()(const T& value) const
  {
    json_value_ = value;
  }
  void operator()(const long int& value) const
  {
    json_value_ = (int)value;
  }
  void operator()(const unsigned long int& value) const
  {
    json_value_ = (unsigned int)value;
  }
  void operator()(const ros::Time& value) const
  {
    json_value_["sec"] = value.sec;
    json_value_["nsec"] = value.nsec;
  }
  void operator()(const ros::Duration& value) const
  {
    json_value_["sec"] = value.sec;
    json_value_["nsec"] = value.nsec;
  }
  void operator()(const roscpp_message_reflection::Message& value) const
  {
    BOOST_FOREACH(const roscpp_message_reflection::Message::FieldEntry& entry, value) {
      entry.value.visit(JsonValueAssignmentVisitor(json_value_[entry.name]));
    }
  }
  template <typename T>
  void operator()(const roscpp_message_reflection::ValueArray<T>& array) const
  {
    for(Json::ArrayIndex i = 0; i < (Json::ArrayIndex)array.size(); ++i) {
      JsonValueAssignmentVisitor root_visitor(json_value_[i]);
      root_visitor(array[i]);
    }
  }
  void operator()(const roscpp_message_reflection::MessageArray& array) const
  {
    for(Json::ArrayIndex i = 0; i < (Json::ArrayIndex)array.size(); ++i) {
      JsonValueAssignmentVisitor root_visitor(json_value_[i]);
      root_visitor(array[i]);
    }
  }
private:
  Json::Value& json_value_;
};

class JsonValueAssignerVisitor : public boost::static_visitor<> {
public:
  JsonValueAssignerVisitor(const Json::Value& json_value) : json_value_(json_value) {}

  void operator()(int8_t& value) const
  {
    value = json_value_.asInt();
  }
  void operator()(uint8_t& value) const
  {
    value = json_value_.asUInt();
  }
  void operator()(int16_t& value) const
  {
    value = json_value_.asInt();
  }
  void operator()(uint16_t& value) const
  {
    value = json_value_.asUInt();
  }
  void operator()(int32_t& value) const
  {
    value = json_value_.asInt();
  }
  void operator()(uint32_t& value) const
  {
    value = json_value_.asUInt();
  }
  void operator()(int64_t& value) const
  {
    value = json_value_.asInt();
  }
  void operator()(uint64_t& value) const
  {
    value = json_value_.asUInt();
  }
  void operator()(float& value) const
  {
    value = json_value_.asFloat();
  }
  void operator()(double& value) const
  {
    value = json_value_.asDouble();
  }
  void operator()(std::string& value) const
  {
    value = json_value_.asString();
  }
  void operator()(ros::Time& value) const
  {
    value.sec = json_value_["sec"].asDouble();
    value.nsec = json_value_["nsec"].asDouble();
  }
  void operator()(ros::Duration& value) const
  {
    value.sec = json_value_["sec"].asDouble();
    value.nsec = json_value_["nsec"].asDouble();
  }
  void operator()(roscpp_message_reflection::Message& value) const
  {
    BOOST_FOREACH(roscpp_message_reflection::Message::FieldEntry& entry, value) {
      entry.value.visit(JsonValueAssignerVisitor(json_value_[entry.name]));
    }
  }
  template <typename T>
  void operator()(roscpp_message_reflection::ValueArray<T>& array) const
  {
    for(Json::ArrayIndex i = 0; i < (Json::ArrayIndex)array.size(); ++i) {
      JsonValueAssignerVisitor root_visitor(json_value_[i]);
      root_visitor(array[i]);
    }
  }
  void operator()(roscpp_message_reflection::MessageArray& array) const
  {
    for(Json::ArrayIndex i = 0; i < (Json::ArrayIndex)array.size(); ++i) {
      JsonValueAssignerVisitor root_visitor(json_value_[i]);
      root_visitor(array[i]);
    }
  }
private:
  const Json::Value& json_value_;
};

JsonRosbridgeProtocolHandler::JsonRosbridgeProtocolHandler(roscpp_message_reflection::NodeHandle& nh,
							   RosbridgeTransport *transport)
  : RosbridgeProtocolHandlerBase(nh, transport) {}

JsonRosbridgeProtocolHandler::~JsonRosbridgeProtocolHandler() {}

void JsonRosbridgeProtocolHandler::onSubscribeCallback(const std::string& topic,
					     const boost::shared_ptr<const roscpp_message_reflection::Message>& msg) {
  Json::Value json_msg;
  json_msg["op"] = "publish";
  json_msg["topic"] = topic;

  JsonValueAssignmentVisitor root_visitor(json_msg["msg"]);
  root_visitor(*msg);

  sendMessage(json_msg);
}

void JsonRosbridgeProtocolHandler::sendStatusMessage(StatusLevel level, const std::string& msg) {
  Json::Value json_msg;
  json_msg["level"] = levelToString(level);
  json_msg["msg"] = msg;
  sendMessage(json_msg);
}

void JsonRosbridgeProtocolHandler::sendMessage(const Json::Value& msg) {
  Json::FastWriter writer;
  boost::shared_ptr<RosbridgeTransport> transport = transport_;
  if(transport)
    transport->sendMessage(writer.write(msg));
}

void JsonRosbridgeProtocolHandler::onMessage(const Buffer& buf) {
  Json::Value msg;
  Json::Reader reader;
  if(reader.parse(buf.begin(), buf.end(), msg, false)) {
    onMessage(msg);
  }
  else {
    StatusMessageStream(this, ERROR) << "Error parsing message: " << reader.getFormattedErrorMessages();
  }
}

void JsonRosbridgeProtocolHandler::onMessage(const Json::Value& json_msg) {
  std::string op = json_msg["op"].asString();
  if(op == "advertise") {
    advertise(json_msg["topic"].asString(), json_msg["type"].asString());
  }
  else if(op == "unadvertise") {
    unadvertise(json_msg["topic"].asString());
  }
  else if(op == "publish") {
    std::string topic = json_msg["topic"].asString();
    roscpp_message_reflection::Publisher pub = getPublisher(topic);
    if(pub) {
      roscpp_message_reflection::Message msg = pub.createMessage();
      JsonValueAssignerVisitor root_visitor(json_msg["msg"]);
      root_visitor(msg);
      pub.publish(msg);
    }
    else {
      StatusMessageStream(this, WARNING) << topic << " is not advertised";
    }
  }
  else if(op == "subscribe") {
    subscribe(json_msg["topic"].asString(), json_msg["type"].asString());
  }
  else if(op == "unsubscribe") {
    unsubscribe(json_msg["topic"].asString());
  }
  else {
    StatusMessageStream(this, ERROR) << "Unsupported operation: " << op;
  }
}


}
