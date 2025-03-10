

#include "uRos_fns.h"
extern rcl_node_t node;
extern uRos_s testSetup;
extern bool ackermann_recv;
#include "../hardware_fns/hardware_fns.h"
#include "../AAV_fns/AAV_fns.h"
#include "../Control_Config.h"
#include "../SteeringBraking/MotorCtrl.h"
#ifdef TRANSPORT_WIFI
int uRos_init_wireless_node_ackermann(uRos_s *uRosStruct, rclc_subscription_callback_t subscription_callback, ackermann_msgs__msg__AckermannDrive *msg, char *ssid, char *pass, int port, char *nodeName, char *topicName, char *pubTopicName){
    rcl_ret_t status;
    IPAddress local_IP(192, 168, 1, 147);

    while (!WiFi.isConnected()) {
        delay(250);
        digitalWrite(BLUE_LED_PIN, LOW);
        delay(250);
        set_microros_wifi_transports(ssid, pass, local_IP, (uint16_t)port);
        if (!WiFi.isConnected()) {
            USER_SERIAL.println("WIFI CONNECTION FAILED, trying again");
        }
        digitalWrite(BLUE_LED_PIN, HIGH);
    }
    USER_SERIAL.println("WIFI CONNECTED");
    digitalWrite(GREEN_LED_PIN, HIGH);

    uRosStruct->allocator = rcl_get_default_allocator();
    status = rclc_support_init(&uRosStruct->support, 0, NULL, &uRosStruct->allocator);
    status = rclc_node_init_default(&uRosStruct->node, nodeName, "", &uRosStruct->support);
    
    // Create subscriber
    status = rclc_subscription_init_best_effort(&uRosStruct->subscriber_1, &uRosStruct->node, ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDrive), topicName);
    
    // Create publisher
    status = rclc_publisher_init_default(&uRosStruct->publisher, &uRosStruct->node, ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDrive), pubTopicName);

    // Create executor
    status = rclc_executor_init(&uRosStruct->executor, &uRosStruct->support.context, 2, &uRosStruct->allocator);
    rclc_executor_add_subscription(&uRosStruct->executor, &uRosStruct->subscriber_1, msg, *subscription_callback, ON_NEW_DATA);
    return status;
}
#endif

#ifdef TRANSPORT_SERIAL
int uRos_init_serial_node_ackermann(uRos_s *uRosStruct, rclc_subscription_callback_t subscription_callback, ackermann_msgs__msg__AckermannDrive *msg, char *nodeName, char *topicName, char *pubTopicName){
    rcl_ret_t status;
    ROS_SERIAL.begin(115200, SERIAL_8N1, RX_PIN_1, TX_PIN_1);
    set_microros_serial_transports(ROS_SERIAL);
    
    uRosStruct->allocator = rcl_get_default_allocator();
    status = rclc_support_init(&uRosStruct->support, 0, NULL, &uRosStruct->allocator);
    status = rclc_node_init_default(&uRosStruct->node, nodeName, "", &uRosStruct->support);
    
    // Create subscriber
    status = rclc_subscription_init_best_effort(&uRosStruct->subscriber_1, &uRosStruct->node, ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDrive), topicName);
    
    // Create publisher
    status = rclc_publisher_init_default(&uRosStruct->publisher_1, &uRosStruct->node, ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDrive), pubTopicName);
    
    // Create executor
    status = rclc_executor_init(&uRosStruct->executor, &uRosStruct->support.context, 2, &uRosStruct->allocator);
    rclc_executor_add_subscription(&uRosStruct->executor, &uRosStruct->subscriber_1, msg, *subscription_callback, ON_NEW_DATA);
    return status;
}
#endif


// #include "../hardware_fns/hardware_fns.h"
// extern Adafruit_MCP4725 dac;





void microROS_Task_sub(void* parameter) {
  pinMode(DRIVER_ERROR_PIN, INPUT);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = uROS_TASK_DELAY / portTICK_PERIOD_MS;
  //dac.setVoltage(1, false);
    rcl_ret_t status;
    while (true) {
        //USER_SERIAL.println("braketask");
        ool rc = !readSwitch(USE_RC, false);
        delay(1);
        if(!readSwitch(USE_RC, false) && rc){
            USER_SERIAL.println("uROS TASK");
            status = rclc_executor_spin_some(&testSetup.executor, RCL_MS_TO_NS(5));
            
        } else{

            USER_SERIAL.println("RC TASK");
            RC_Control();
        }

        if(digitalRead(DRIVER_ERROR_PIN)){
                USER_SERIAL.println("DRIVER ERROR");
            }
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait until next cycle
    }
}

extern CurrState state;
void microROS_Task_pub(void* parameter) {
    pinMode(DRIVER_ERROR_PIN, INPUT);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = uROS_TASK_DELAY / portTICK_PERIOD_MS;
    //dac.setVoltage(1, false);
      rcl_ret_t status;
      while (true) {
        ackermann_msgs__msg__AckermannDrive msg_sub;

        ackermann_recv = false;
        msg_sub.steering_angle = state.steer_angle;
        msg_sub.acceleration = state.accel;
        
        status = rcl_publish(&testSetup.publisher, &msg_sub, NULL);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait until next cycle
      }
  }





