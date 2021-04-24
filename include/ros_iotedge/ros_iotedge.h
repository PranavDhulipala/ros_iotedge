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


class ROSIoTEdge{

public:

  ROSIoTEdge(ros::NodeHandle* n):node_(*n){};

  static IOTHUBMESSAGE_DISPOSITION_RESULT InputQueue1Callback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback);

  static IOTHUB_MODULE_CLIENT_LL_HANDLE InitializeConnection();

  static void DeInitializeConnection(IOTHUB_MODULE_CLIENT_LL_HANDLE iotHubModuleClientHandle);

  static int SetupCallbacksForModule(IOTHUB_MODULE_CLIENT_LL_HANDLE iotHubModuleClientHandle);

  void iothub_module();

  ~ROSIoTEdge(){};

private:

  size_t messagesReceivedByInput1Queue = 0;

  ros::NodeHandle node_;

  ros::Publisher bbox_publisher_;

};

#endif
