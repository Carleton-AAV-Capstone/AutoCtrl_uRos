
#include "../hardware_config.h"
#include "../rc.h"
#include "uRos_fns.h"

extern rcl_node_t node;
extern uRos_s testSetup;
extern bool ackermann_recv;
extern CurrState curr_state;
// #include "../hardware_fns.h"
// #include "../AAV_fns/AAV_fns.h"
 ///#include "../Control_Config.h"
// #include "../SteeringBraking/MotorCtrl.h"
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
int uRos_init_serial_node_ackermann(uRos_s *uRosStruct, 
    rclc_subscription_callback_t ackermann_callback, 
    ackermann_msgs__msg__AckermannDrive *ackermann_msg, 
    rclc_subscription_callback_t gps_callback,
    sensor_msgs__msg__NavSatFix *gps_msg,
    geometry_msgs__msg__TwistStamped *gps_twist,
    rclc_subscription_callback_t gps_twist_callback,
    char *nodeName, char *ackermannTopicName, 
    char *pubTopicName, char *gpsTopicName, char *gps_twistTopicName) {
rcl_ret_t status;
ROS_SERIAL.begin(UROS_BAUD, SERIAL_8N1, UROS_RX, UROS_TX);
set_microros_serial_transports(ROS_SERIAL);

uRosStruct->allocator = rcl_get_default_allocator();

status = rclc_support_init(&uRosStruct->support, 0, NULL, &uRosStruct->allocator);
if (status != RCL_RET_OK) {
USER_SERIAL.println("Failed to initialize support");
return status;
}

status = rclc_node_init_default(&uRosStruct->node, nodeName, "auto_node", &uRosStruct->support);
if (status != RCL_RET_OK) {
USER_SERIAL.println("Failed to initialize node");
return status;
}

// Create Ackermann subscriber
status = rclc_subscription_init_best_effort(&uRosStruct->subscriber_1, &uRosStruct->node, 
ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDrive), ackermannTopicName);
if (status != RCL_RET_OK) {
USER_SERIAL.println("Failed to initialize Ackermann subscriber");
return status;
}

// Create GPS subscriber
status = rclc_subscription_init_best_effort(&uRosStruct->subscriber_2, &uRosStruct->node, 
ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), gpsTopicName);
if (status != RCL_RET_OK) {
USER_SERIAL.println("Failed to initialize GPS subscriber");
return status;
}

status = rclc_subscription_init_best_effort(&uRosStruct->subscriber_3, &uRosStruct->node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), gps_twistTopicName);
    if (status != RCL_RET_OK) {
    USER_SERIAL.println("Failed to initialize GPS subscriber");
    return status;
    }

// Create Ackermann publisher
status = rclc_publisher_init_default(&uRosStruct->publisher_1, &uRosStruct->node, 
ROSIDL_GET_MSG_TYPE_SUPPORT(ackermann_msgs, msg, AckermannDrive), pubTopicName);
if (status != RCL_RET_OK) {
USER_SERIAL.println("Failed to initialize publisher");
return status;
}

// Create executor
status = rclc_executor_init(&uRosStruct->executor, &uRosStruct->support.context, 3, &uRosStruct->allocator);
if (status != RCL_RET_OK) {
USER_SERIAL.println("Failed to initialize executor");
return status;
}

// Add Ackermann subscription to executor
status = rclc_executor_add_subscription(&uRosStruct->executor, &uRosStruct->subscriber_1, ackermann_msg, *ackermann_callback, ON_NEW_DATA);
if (status != RCL_RET_OK) {
USER_SERIAL.println("Failed to add Ackermann subscription to executor");
return status;
}

// Add GPS subscription to executor
status = rclc_executor_add_subscription(&uRosStruct->executor, &uRosStruct->subscriber_2, gps_msg, *gps_callback, ON_NEW_DATA);
if (status != RCL_RET_OK) {
USER_SERIAL.println("Failed to add GPS subscription to executor");
return status;
}

// Add GPS twist subscription to executor
status = rclc_executor_add_subscription(&uRosStruct->executor, &uRosStruct->subscriber_3, gps_twist, *gps_twist_callback, ON_NEW_DATA);
if (status != RCL_RET_OK) {
USER_SERIAL.println("Failed to add GPS subscription to executor");
return status;
}

return status;
}
#endif


// #include "../hardware_fns/hardware_fns.h"
// extern Adafruit_MCP4725 dac;




void microROS_Task_sub(void* parameter) {
  pinMode(DRIVER_ERROR_PIN, INPUT);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = uROS_TASK_DELAY_SUB / portTICK_PERIOD_MS;
  //dac.setVoltage(1, false);
    rcl_ret_t status;
    while (true) {
        //USER_SERIAL.println("braketask");
        bool rc = !readSwitch(USE_RC, false);
        USER_SERIAL.print("RC: ");
        USER_SERIAL.println(readChannel(USE_RC, 0, 100, 100));
        vTaskDelay(10);
        if(!readSwitch(USE_RC, false) && rc){
            USER_SERIAL.println("uROS TASK");
            status = rclc_executor_spin_some(&testSetup.executor, RCL_MS_TO_NS(5));
            
        } else{

            USER_SERIAL.println("RC TASK");
            RC_Control();
        }

        // if(digitalRead(DRIVER_ERROR_PIN)){
        //         USER_SERIAL.println("DRIVER ERROR");
        //     }
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait until next cycle
    }
}

extern JrkG2I2C jrk_steer;
void microROS_Task_pub(void* parameter) {
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = uROS_TASK_DELAY_PUB / portTICK_PERIOD_MS;
    //dac.setVoltage(1, false);
      rcl_ret_t status;
      while (true) {
        ackermann_msgs__msg__AckermannDrive msg_sub;
        
        ackermann_recv = false;
        msg_sub.steering_angle = (float) map((long)jrk_steer.getScaledFeedback(), 0, 4095, 100, -100)/100.0;
        USER_SERIAL.print("Steering angle carla: ");
        USER_SERIAL.println(msg_sub.steering_angle);
        msg_sub.acceleration = 1;
        msg_sub.speed = curr_state.accel/2000;
        
        status = rcl_publish(&testSetup.publisher_1, &msg_sub, NULL);
        USER_SERIAL.println("PUBLISHED");

        

        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait until next cycle
      }
  }


void uROS_Clock_Sync(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = CLOCK_SYNC_TIME / portTICK_PERIOD_MS;
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait until next cycle
        rclc_executor_spin_some(&testSetup.executor, RCL_MS_TO_NS(5));
    }
}





