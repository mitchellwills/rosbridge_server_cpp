cmake_minimum_required(VERSION 2.8.3)
project(rosbridge_server_cpp)

find_package(catkin REQUIRED COMPONENTS
  async_web_server_cpp
  roscpp
  roscpp_message_reflection
)

find_package(Boost REQUIRED)

find_package(PNG REQUIRED)

# JsonCpp
set(JsonCpp_INCLUDE_DIRS /usr/include/jsoncpp)
set(JsonCpp_LIBRARIES jsoncpp)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rosbridge_server_cpp
 CATKIN_DEPENDS async_web_server_cpp roscpp roscpp_message_reflection
 DEPENDS Boost
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${JsonCpp_INCLUDE_DIRS}
  ${PNG_INCLUDE_DIRS}
)


add_library(rosbridge_server_cpp
  src/rosbridge_server.cpp
  src/rosbridge_transport.cpp
  src/rosbridge_websocket_transport.cpp
  src/rosbridge_protocol_handler.cpp
  src/json_rosbridge_protocol_handler.cpp
  src/png_image.cpp
  src/base64.cpp
)
target_link_libraries(rosbridge_server_cpp
  ${catkin_LIBRARIES}
  ${JsonCpp_LIBRARIES}
  ${PNG_LIBRARIES}
)


add_executable(rosbridge_websocket_server src/rosbridge_websocket_server.cpp)
target_link_libraries(rosbridge_websocket_server
  rosbridge_server_cpp
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS rosbridge_websocket_server rosbridge_server_cpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rosbridge_server_cpp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
