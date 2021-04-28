#ifndef ROS_IOTEDGE_H
#define ROS_IOTEDGE_H

#include <ros/ros.h>
#include "ros_iotedge/BBox.h"

#include <stdlib.h>

#include "iothub_module_client_ll.h"
#include "iothub_client_options.h"
#include "iothub_message.h"
#include "azure_c_shared_utility/threadapi.h"
#include "azure_c_shared_utility/crt_abstractions.h"
#include "azure_c_shared_utility/platform.h"
#include "azure_c_shared_utility/shared_util_options.h"
#include "iothubtransportmqtt.h"
#include "iothub.h"
#include "time.h"
#include <parson.h>
#include <geometry_msgs/Twist.h>


class ROSIoTEdge{

public:

  explicit ROSIoTEdge(ros::NodeHandle* n);

  static IOTHUBMESSAGE_DISPOSITION_RESULT InputQueue1Callback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback);

  static IOTHUBMESSAGE_DISPOSITION_RESULT InputQueue2Callback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback);

  IOTHUB_MODULE_CLIENT_LL_HANDLE InitializeConnection();

  void DeInitializeConnection(IOTHUB_MODULE_CLIENT_LL_HANDLE iotHubModuleClientHandle);

  int SetupCallbacksForModule(IOTHUB_MODULE_CLIENT_LL_HANDLE iotHubModuleClientHandle);

  void iothub_module();

  void PublishFromEyeMsg(unsigned const char* message);

  void PublishFromSpeechMsg(unsigned const char* message);

  ~ROSIoTEdge();

private:

  ros::NodeHandle node_;

  ros::Publisher bbox_publisher_;

  ros::Publisher vel_publisher_;

  double rate_ = 10.0;

  double move_time_ = 6.0;

  double turn_time_ = 3.0;

  static ROSIoTEdge* ptr;

};

ROSIoTEdge* ROSIoTEdge::ptr = nullptr;

#endif
