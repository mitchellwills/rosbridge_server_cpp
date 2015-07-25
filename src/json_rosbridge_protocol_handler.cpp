#include "rosbridge_server_cpp/json_rosbridge_protocol_handler.h"

namespace rosbridge_server_cpp {

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
  json_msg["op"] = "status";
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
  else if(op == "set_level") {
    setStatusLevel(stringToLevel(json_msg["level"].asString()));
  }
  else {
    StatusMessageStream(this, ERROR) << "Unsupported operation: " << op;
  }
}


}
