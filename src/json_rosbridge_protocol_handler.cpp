#include "rosbridge_server_cpp/json_rosbridge_protocol_handler.h"
#include "rosbridge_server_cpp/png_image.h"
#include "rosbridge_server_cpp/base64.h"

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
    json_value_ = Json::Value(Json::objectValue);
    BOOST_FOREACH(const roscpp_message_reflection::Message::FieldEntry& entry, value) {
      entry.value.visit(JsonValueAssignmentVisitor(json_value_[entry.name]));
    }
  }
  template <typename T>
  void operator()(const roscpp_message_reflection::ValueArray<T>& array) const
  {
    json_value_ = Json::Value(Json::arrayValue);
    for(Json::ArrayIndex i = 0; i < (Json::ArrayIndex)array.size(); ++i) {
      JsonValueAssignmentVisitor root_visitor(json_value_[i]);
      root_visitor(array[i]);
    }
  }
  void operator()(const roscpp_message_reflection::MessageArray& array) const
  {
    json_value_ = Json::Value(Json::arrayValue);
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
      if(json_value_.isMember(entry.name)) {
	entry.value.visit(JsonValueAssignerVisitor(json_value_[entry.name]));
      }
    }
  }
  template <typename T>
  void operator()(roscpp_message_reflection::ValueArray<T>& array) const
  {
    array.resize(array.size());
    for(Json::ArrayIndex i = 0; i < (Json::ArrayIndex)array.size(); ++i) {
      JsonValueAssignerVisitor root_visitor(json_value_[i]);
      root_visitor(array[i]);
    }
  }
  void operator()(roscpp_message_reflection::MessageArray& array) const
  {
    array.resize(array.size());
    for(Json::ArrayIndex i = 0; i < (Json::ArrayIndex)array.size(); ++i) {
      JsonValueAssignerVisitor root_visitor(json_value_[i]);
      root_visitor(array[i]);
    }
  }
private:
  const Json::Value& json_value_;
};

JsonRosbridgeProtocolHandler::JsonRosbridgeProtocolHandler(roscpp_message_reflection::NodeHandle& nh,
							   boost::shared_ptr<RosbridgeTransport>& transport)
  : RosbridgeProtocolHandlerBase(nh, transport) {}

JsonRosbridgeProtocolHandler::~JsonRosbridgeProtocolHandler() {}

void JsonRosbridgeProtocolHandler::onSubscribeCallback(const std::string& topic,
						       const MessageSendOptions& options,
						       const boost::shared_ptr<const roscpp_message_reflection::Message>& msg) {
  Json::Value json_msg;
  json_msg["op"] = "publish";
  json_msg["topic"] = topic;

  JsonValueAssignmentVisitor root_visitor(json_msg["msg"]);
  root_visitor(*msg);

  sendMessage(json_msg, options);
}

void JsonRosbridgeProtocolHandler::sendStatusMessage(StatusLevel level, const std::string& id, const std::string& msg) {
  Json::Value json_msg;
  json_msg["op"] = "status";
  json_msg["level"] = levelToString(level);
  json_msg["msg"] = msg;
  if(!id.empty())
    json_msg["id"] = id;
  sendMessage(json_msg, MessageSendOptions());
}

void JsonRosbridgeProtocolHandler::sendServiceServerRequest(const std::string& service, const std::string& id,
							    const ServiceServerOptions& options,
							    const roscpp_message_reflection::Message& request) {
  Json::Value json_msg;
  json_msg["op"] = "call_service";
  json_msg["service"] = service;
  JsonValueAssignmentVisitor root_visitor(json_msg["args"]);
  root_visitor(request);
  if(!id.empty())
    json_msg["id"] = id;
  sendMessage(json_msg, MessageSendOptions());
}


static bool pngCompress(const Buffer& data, std::string* output) {
  size_t width = floor(sqrt(data.size()/3.0));
  size_t height = ceil((data.size()/3.0) / width);

  PngImage image;
  image.fromBuffer(width, height, data, '\n');
  std::vector<char> png_image;
  if(!image.write(&png_image))
    return false;

  base64Encode(png_image, output);
  return true;
}

void JsonRosbridgeProtocolHandler::sendMessage(const Json::Value& msg, const MessageSendOptions& options) {
  Json::FastWriter writer;
  std::string msg_str = writer.write(msg);
  if(options.compression == "none") {
    transport_->sendMessage(msg_str);
  }
  else if(options.compression == "png") {
    Json::Value json_msg;
    json_msg["op"] = "png";
    std::string compress_buf;
    pngCompress(msg_str, &compress_buf);
    json_msg["data"] = compress_buf;
    sendMessage(json_msg, MessageSendOptions());
  }
  else {
    StatusMessageStream(this, ERROR, "") << "Unsupported compression method: " << options.compression;
  }
}

void JsonRosbridgeProtocolHandler::onMessage(const Buffer& buf) {
  Json::Value msg;
  Json::Reader reader;
  if(reader.parse(buf.begin(), buf.end(), msg, false)) {
    onMessage(msg);
  }
  else {
    StatusMessageStream(this, ERROR, "") << "Error parsing message: " << reader.getFormattedErrorMessages();
  }
}

void JsonRosbridgeProtocolHandler::onMessage(const Json::Value& json_msg) {
  std::string op = json_msg["op"].asString();
  std::string id = json_msg["id"].asString();
  if(op == "advertise") {
    advertise(json_msg["topic"].asString(), json_msg["type"].asString(), id, PublishOptions());
  }
  else if(op == "unadvertise") {
    unadvertise(json_msg["topic"].asString(), id);
  }
  else if(op == "publish") {
    std::string topic = json_msg["topic"].asString();
    roscpp_message_reflection::Publisher pub = getPublisher(topic);
    if(pub) {
      roscpp_message_reflection::Message msg = pub.createMessage();
      JsonValueAssignerVisitor root_visitor(json_msg["msg"]);
      root_visitor(msg);
      // TODO handle publish special cases
      pub.publish(msg);
    }
    else {
      StatusMessageStream(this, ERROR, id) << topic << " is not advertised";
    }
  }
  else if(op == "subscribe") {
    SubscribeOptions options;
    options.message_send.compression = json_msg["compression"].asString();
    subscribe(json_msg["topic"].asString(), json_msg["type"].asString(), id, options);
  }
  else if(op == "unsubscribe") {
    unsubscribe(json_msg["topic"].asString(), id);
  }
  else if(op == "call_service") {
    std::string service = json_msg["service"].asString();
    std::string type = json_msg["type"].asString();
    std::string id = json_msg["id"].asString();
    roscpp_message_reflection::ServiceClient client = getServiceClient(service, type);
    if(client) {
      roscpp_message_reflection::Message request = client.createRequestMessage();
      JsonValueAssignerVisitor request_root_visitor(json_msg["args"]);
      request_root_visitor(request);
      roscpp_message_reflection::Message response = client.createResponseMessage();
      bool result = client.call(request, &response);

      // Send back the result
      Json::Value json_response_msg;
      json_response_msg["op"] = "service_response";
      json_response_msg["service"] = service;
      json_response_msg["id"] = id;
      json_response_msg["result"] = result;
      JsonValueAssignmentVisitor response_root_visitor(json_response_msg["values"]);
      response_root_visitor(response);
      sendMessage(json_response_msg, MessageSendOptions());
    }
    else {
      StatusMessageStream(this, ERROR, id) << "Could not create service client: " << service;
    }
  }
  else if(op == "advertise_service") {
    ServiceServerOptions options;
    advertiseService(json_msg["service"].asString(), json_msg["type"].asString(), id, options);
  }
  else if(op == "unadvertise_service") {
    unadvertiseService(json_msg["service"].asString(), id);
  }
  else if(op == "service_response") {
    std::string service = json_msg["service"].asString();
    std::string id = json_msg["id"].asString();
    PendingServiceCallResolver resolver(this, service, id);
    if(resolver.isActive()) {
      JsonValueAssignerVisitor root_visitor(json_msg["values"]);
      root_visitor(resolver.getResponseMessage());
      resolver.resolve(json_msg["result"].asBool());
    }
    else {
      StatusMessageStream(this, ERROR, id) << service << " no pending call found for call id: " << id;
    }
  }
  else if(op == "set_level") {
    std::string level_str = json_msg["level"].asString();
    StatusLevel level = stringToLevel(level_str);
    if(level == INVALID_LEVEL) {
      StatusMessageStream(this, ERROR, id) << "Bad status level: " << level_str;
    }
    else {
      setStatusLevel(level, id);
    }
  }
  else {
    StatusMessageStream(this, ERROR, id) << "Unsupported operation: " << op;
  }
}


}
