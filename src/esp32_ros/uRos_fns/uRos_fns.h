
#include <Arduino.h>

#include "../hardware_fns/hardware_config.h"
#include <micro_ros_platformio.h>
#include <Adafruit_MCP4725.h>
//#include <micro_ros_arduino.h>
#include <stdlib.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rmw/rmw.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/init.h>
#include <rclc/executor.h>
#include <platform_code/micro_ros_platformio.h>




#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joy.h>
#include <aav_drive_msg/msg/drive.h>
#include <ackermann_msgs/msg/ackermann_drive.h>





/*_____________________________________MACROS_____________________________________________________*/
#define uROS_IP (192, 168, 1, 147)
#define uROS_PORT 8888
#define NET_SSID "AAVwifi"
#define NET_PASS "aav@2023"

typedef struct uRos_struct_s{
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_subscription_t subscriber_1;
    rcl_subscription_t subscriber_2;
    rclc_executor_t executor;
    rclc_executor_t executor_2;

}uRos_s;
int uRos_init_wireless_node_ackermann(uRos_s *uRosStruct, rclc_subscription_callback_t subscription_callback, ackermann_msgs__msg__AckermannDrive *msg, char *ssid, char *pass, int port, char *nodeName, char *topicName);
int uRos_init_serial_node_ackermann(uRos_s *uRosStruct, rclc_subscription_callback_t subscription_callback, ackermann_msgs__msg__AckermannDrive *msg, char *nodeName, char *topicName);
void microROS_Task(void* parameter);
void microROS_Task_joy(void* parameter);
void microROS_Task_throttle(void* parameter);