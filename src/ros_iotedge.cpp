#include "ros_iotedge/ros_iotedge.h"


ROSIoTEdge::ROSIoTEdge(ros::NodeHandle* n):node_(*n)
{
    bbox_publisher_ = node_.advertise<ros_iotedge::BBox>("bounding_box", 1000);
    ptr = this;
}

IOTHUBMESSAGE_DISPOSITION_RESULT ROSIoTEdge::InputQueue1Callback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback)
{
    IOTHUBMESSAGE_DISPOSITION_RESULT result;
    IOTHUB_MODULE_CLIENT_LL_HANDLE iotHubModuleClientHandle = (IOTHUB_MODULE_CLIENT_LL_HANDLE)userContextCallback;

    unsigned const char* messageBody;
    size_t contentSize;

    if (IoTHubMessage_GetByteArray(message, &messageBody, &contentSize) != IOTHUB_MESSAGE_OK)
    {
        messageBody = reinterpret_cast<const unsigned char *>("<null>");
    }

    ROS_INFO("Data: [%s]", messageBody);

    ptr->PublishFromMsg(messageBody);

    result = IOTHUBMESSAGE_ACCEPTED;

    return result;
}

IOTHUB_MODULE_CLIENT_LL_HANDLE ROSIoTEdge::InitializeConnection()
{
    IOTHUB_MODULE_CLIENT_LL_HANDLE iotHubModuleClientHandle;

    if (IoTHub_Init() != 0)
    {
        ROS_DEBUG("Failed to initialize the platform.");
        iotHubModuleClientHandle = NULL;
    }
    else if ((iotHubModuleClientHandle = IoTHubModuleClient_LL_CreateFromEnvironment(MQTT_Protocol)) == NULL)
    {
        ROS_ERROR("ERROR: IoTHubModuleClient_LL_CreateFromEnvironment failed");
    }
    else
    {
        // Uncomment the following lines to enable verbose logging.
        // bool traceOn = true;
        // IoTHubModuleClient_LL_SetOption(iotHubModuleClientHandle, OPTION_LOG_TRACE, &trace);
    }

    return iotHubModuleClientHandle;
}

void ROSIoTEdge::DeInitializeConnection(IOTHUB_MODULE_CLIENT_LL_HANDLE iotHubModuleClientHandle)
{
    if (iotHubModuleClientHandle != NULL)
    {
        IoTHubModuleClient_LL_Destroy(iotHubModuleClientHandle);
    }
    IoTHub_Deinit();
}

int ROSIoTEdge::SetupCallbacksForModule(IOTHUB_MODULE_CLIENT_LL_HANDLE iotHubModuleClientHandle)
{
    int ret;

    if (IoTHubModuleClient_LL_SetInputMessageCallback(iotHubModuleClientHandle, "AzureEyeModuleInput", InputQueue1Callback, (void*) iotHubModuleClientHandle) != IOTHUB_CLIENT_OK)
    {
        ROS_ERROR("ERROR: IoTHubModuleClient_LL_SetInputMessageCallback(\"input1\")..........FAILED!");
        ret = 1;
    }
    else
    {
        ret = 0;
    }

    return ret;
}

void ROSIoTEdge::iothub_module()
{
    IOTHUB_MODULE_CLIENT_LL_HANDLE iotHubModuleClientHandle;

    srand((unsigned int)time(NULL));

    if ((iotHubModuleClientHandle = InitializeConnection()) != NULL && SetupCallbacksForModule(iotHubModuleClientHandle) == 0)
    {
        // The receiver just loops constantly waiting for messages.
        ROS_INFO("Waiting for incoming messages.");
        while (true)
        {
            IoTHubModuleClient_LL_DoWork(iotHubModuleClientHandle);
            // ros::spin();
            ThreadAPI_Sleep(100);
        }
    }

    DeInitializeConnection(iotHubModuleClientHandle);
}

void ROSIoTEdge::PublishFromMsg(unsigned const char* message)
{
    ros_iotedge::BBox msg;
    JSON_Value *root_value = json_parse_string((const char *)message);
    JSON_Object *root_object = json_value_get_object(root_value);
    if (root_object)
    {
      JSON_Array *nn_array = json_object_get_array(root_object, "NEURAL_NETWORK");
      if (nn_array)
      {
        size_t sz = json_array_get_count(nn_array);
        if (sz)
        {
          JSON_Object *nn_object = json_array_get_object(nn_array, 0);
          JSON_Array *bbox_array = json_object_get_array(nn_object, "bbox");
          if (bbox_array)
          {
            size_t count = json_array_get_count(bbox_array);
            for (size_t i = 0; i < count; i++)
            {
              float value = (float)json_array_get_number(bbox_array, i);
              msg.box.push_back(value);
            }
          }
          std::string label = json_object_dotget_string(nn_object, "label");
          msg.header.frame_id = label;
          float confidence = std::stof(json_object_dotget_string(nn_object, "confidence"));
          msg.confidence = confidence;
          std::string timestamp = json_object_dotget_string(nn_object, "timestamp");
          std::string sub1 = timestamp.substr(0, 10);
          std::string sub2 = timestamp.substr(10);
          ros::Time time(std::stoul(sub1), std::stoul(sub2));
          msg.header.stamp = time;
          bbox_publisher_.publish(msg);
        }
      }
    }
        
}

ROSIoTEdge::~ROSIoTEdge(){};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_iotedge");
    ros::NodeHandle nh;
    ROSIoTEdge edge(&nh);
    edge.iothub_module();
    return 0;
}
