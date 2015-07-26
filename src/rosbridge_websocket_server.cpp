#include "rosbridge_server_cpp/rosbridge_server.h"
#include "rosbridge_server_cpp/rosbridge_websocket_transport.h"

using namespace rosbridge_server_cpp;

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosbridge_websocket_server");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  {
    RosbridgeServer server(nh, new WebsocketTransportServer("0.0.0.0", "9090"));

    ros::spin();
  }

  return 0;
}
