#include "ros_iotedge/ros_iotedge.h"

IOTHUBMESSAGE_DISPOSITION_RESULT ROSIoTEdge::InputQueue1Callback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback)
{
    IOTHUBMESSAGE_DISPOSITION_RESULT result;
    // IOTHUB_CLIENT_RESULT clientResult;
    IOTHUB_MODULE_CLIENT_LL_HANDLE iotHubModuleClientHandle = (IOTHUB_MODULE_CLIENT_LL_HANDLE)userContextCallback;

    unsigned const char* messageBody;
    size_t contentSize;

    if (IoTHubMessage_GetByteArray(message, &messageBody, &contentSize) != IOTHUB_MESSAGE_OK)
    {
        messageBody = reinterpret_cast<const unsigned char *>("<null>");
    }

    ROS_INFO("Data: [%s]", messageBody);

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
            ThreadAPI_Sleep(100);
        }
    }

    DeInitializeConnection(iotHubModuleClientHandle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_iotedge");
    ros::NodeHandle nh;
    ROSIoTEdge edge(&nh);
    edge.iothub_module();
    return 0;
}
