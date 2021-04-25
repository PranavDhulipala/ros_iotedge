cmake_minimum_required(VERSION 3.1)
project(ros_iotedge)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_EDGE_MODULES")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_EDGE_MODULES")

set(AZUREIOT_INC_FOLDER "/usr/include/azureiot" "/usr/include/azureiot/inc")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
 FILES
 BBox.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_library(parson STATIC
  parson/parson.c
  parson/parson.h
)

include_directories(
  "include"
  parson
  ${catkin_INCLUDE_DIRS}
  ${AZUREIOT_INC_FOLDER}
)

add_executable(ros_iotedge src/ros_iotedge.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  iothub_client_mqtt_transport
  iothub_client
  parson
  umqtt
  prov_auth_client
  hsm_security_client
  uhttp
  aziotsharedutil
  pthread
  curl
  ssl
  crypto
  m
  uuid
)
