#include "ros/ros.h"

// msgs
#include "std_msgs/Float64.h"
#include "can_bus_ros_version/CanBusMsgStamped.h"

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <vector>

#include "can_bus_ros_version/icsnVC40.h"
#include "can_bus_ros_version/icsneoLinuxAPI.h"
#include "can_bus_ros_version/OCriticalSection.h"

// Default topic names. Should be changed with <remap>
std::string default_steering_pub_topic = "can_steering_wheel_angle_deg";
std::string default_can_pub_topic = "can_msg";

// Default node name
std::string node_name = "can_bus_interface";

// Default parameters
int default_rate = 50;
double default_offset = 0;
bool only_steering = false;
bool print = false;

// CAN Bus
void* hObject = 0;
OCriticalSection csShutdown;
bool bShutDown;

// publish data
double current_steering_wheel_angle;
ros::Publisher pub_can;

// students.asl.ethz.ch/upl_pdf/151-report.pdf
double computeSteeringWheelAngle(int byte0, int byte1)
{
    double decision_value = (double) byte0 * 16 * 16 + byte1 - default_offset;
    double result = 0;

    if(decision_value < 2048)
    {
        result = decision_value * 1.5;
    } else if(decision_value >= 2048)
    {
        result = (decision_value - 4096) * 1.5;
    }

    return result;
} 

void* ReadThread(void* lParam)
{
    int NumErrors = 0;
    int NumMsgs = 0;
    icsSpyMessage Msgs[20000];
    bool bDone = false;

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

    while (1) {
        //check for shutdown from the main thread
        csShutdown.Lock();
        bDone = bShutDown;
        csShutdown.Unlock();

        if (bDone)
            break;

        if (icsneoWaitForRxMessagesWithTimeOut(hObject, 1000) == 0)
            continue;

        if (icsneoGetMessages(hObject, Msgs, &NumMsgs, &NumErrors) == 0)
            continue;

        for (int i = 0; i < NumMsgs; i++) {

            if(Msgs[i].ArbIDOrHeader == 0x25) // Toyota Prius steering wheel angle msg [deg]
            {
                current_steering_wheel_angle = computeSteeringWheelAngle(Msgs[i].Data[0], Msgs[i].Data[1]);
                if(print)
                    printf("steering wheel angle [deg] %f\n", current_steering_wheel_angle);
            }

            if(!only_steering)
            {
                can_bus_ros_version::CanBusMsgStamped can_msg;
                can_msg.header.stamp = ros::Time::now();
                can_msg.network_id = Msgs[i].NetworkID;
                can_msg.msg_id = Msgs[i].ArbIDOrHeader;
                std::vector<int> data_bytes;

                if(print)
                {
                    printf("Network %d ArbID = %lX \t - Data Bytes: ", Msgs[i].NetworkID, Msgs[i].ArbIDOrHeader);
                }

                for (int j = 0; j < Msgs[i].NumberBytesData; j++)
                {
                    if(print)
                    {
                        printf("%02X ", Msgs[i].Data[j]);
                    }

                    data_bytes.push_back(Msgs[i].Data[j]);
                }

                if(print)
                {
                    printf("\n");
                    printf("size data_bytes %d\n", data_bytes.size());
                }

                can_msg.data_bytes = data_bytes;
                pub_can.publish(can_msg);
            }
        }
    }

    printf("ReadThread done\n");

    return 0;
}


int main(int argc, char **argv)
{
    // ROS section

    ROS_INFO("starting can_bus_node");
    ros::init(argc, argv, node_name);
    ros::NodeHandle *n = new ros::NodeHandle();

    int rate;

    ros::NodeHandle private_node_handle("~");
    private_node_handle.param("rate", rate, default_rate);
    private_node_handle.param("offset", default_offset, default_offset);
    private_node_handle.param("only_steering", only_steering, only_steering);
    private_node_handle.param("print", print, print);

    ros::Publisher steering_pub = n->advertise<std_msgs::Float64>(default_steering_pub_topic, 1);
    pub_can = n->advertise<can_bus_ros_version::CanBusMsgStamped>(default_can_pub_topic, 1);

    ros::Rate r(rate);

    // CAN Bus section

    NeoDevice Nd[255];
    int iRetVal = 0;
    int i, serial_to_open = 0, index_to_open;
    int NumDevices = 255;
    int NumErrors;
    icsSpyMessage OutMsg;
    char DeviceType[25];
    char chIn;
    unsigned long DeviceTypes;

    if (argc >= 2 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        printf("icsneo sample application\n-s, --serial\t open a specific serial number\n-h, --help\tprint this message\n");
        return 0;
    }

    if (argc >= 3 && (strcmp(argv[1], "-s") == 0 || strcmp(argv[1], "--serial") == 0))
        serial_to_open = atoi(argv[2]);

    bShutDown = false;

    printf("Starting...\n");

    //DeviceTypes = NEODEVICE_VCAN3;
    DeviceTypes = NEODEVICE_RED;

    iRetVal = icsneoFindNeoDevices(DeviceTypes, Nd, &NumDevices);

    if (iRetVal == 0 || NumDevices == 0)
    {
        printf("No devices found...exiting with ret = %d\n", iRetVal);
        return 0;
    }

    printf("%d Device(s) found\n", NumDevices);

    for (i = 0; i < NumDevices; i++) {
        switch (Nd[i].DeviceType) {

        case NEODEVICE_FIRE:
            strcpy(DeviceType, "Fire");
            break;

        case NEODEVICE_VCAN3:
            strcpy(DeviceType, "ValueCAN3");
            break;

        case NEODEVICE_YELLOW:
            strcpy(DeviceType, "Yellow");
            break;

        case NEODEVICE_RED:
            strcpy(DeviceType, "Red");
            break;

        default:
            strcpy(DeviceType, "unknown device");
        }

        if (Nd[i].SerialNumber == serial_to_open)
            index_to_open = i;

        printf("Device %d: ", i + 1);
        printf("Serial # %d Type = %s\n", Nd[i].SerialNumber, DeviceType);
    }
    if (serial_to_open == 0)
        index_to_open = 0;

    printf("\nOpening device %i\n", index_to_open + 1);

    iRetVal = icsneoOpenNeoDevice(&Nd[index_to_open], &hObject, NULL, 1, 0 /*DEVICE_OPTION_DONT_ENABLE_NETCOMS*/);

    if (iRetVal == 1) {
        printf("Device was opened!\n\n");
    }
    else {
        printf("Failed to open the device\n");
        icsneoShutdownAPI();
        return 0;
    }

    pthread_t threadid;
    pthread_create(&threadid, NULL, &ReadThread, NULL);

    // transmitting not tested
    /**while (!bExit) {
        printf("enter t to transmit\n");
        printf("enter e to exit\n");
        chIn = getc(stdin);

        switch (chIn) {
        case 't':
            icsSpyMessage OutMsg;
            OutMsg.ArbIDOrHeader = 0x500;
            OutMsg.Data[0] = Nd[index_to_open].SerialNumber & 0xff;
            OutMsg.Data[1] = (Nd[index_to_open].SerialNumber >> 8) & 0xff;
            OutMsg.Data[2] = (Nd[index_to_open].SerialNumber >> 16);
            OutMsg.Data[3] = (Nd[index_to_open].SerialNumber >> 24);
            OutMsg.Data[4] = 0xde;
            OutMsg.Data[5] = 0xad;
            OutMsg.Data[6] = 0xbe;
            OutMsg.Data[7] = 0xef;
            OutMsg.NumberBytesData = 8;
            OutMsg.NumberBytesHeader = 2;
            iRetVal = icsneoTxMessages(hObject, &OutMsg, NETID_HSCAN, 1);
            break;

        case 'e':

            bExit = true;

            break;
        }
    }**/

    while (n->ok())
    {
        std_msgs::Float64 steeringMsg;
        steeringMsg.data = current_steering_wheel_angle;
        steering_pub.publish(steeringMsg);

        ros::spinOnce();
        r.sleep();
    }

    printf("Ending...\n");

    csShutdown.Lock();
    bShutDown = true;
    csShutdown.Unlock();
    usleep(1000 * 1000); // let the read thread close

    icsneoClosePort(hObject, &NumErrors);
    icsneoFreeObject(hObject);
    icsneoShutdownAPI();

    printf("Main thread done...\n");

    return 0;

}
