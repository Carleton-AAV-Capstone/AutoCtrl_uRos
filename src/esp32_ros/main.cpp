
#include <Arduino.h>
#include  "AAV_fns/AAV_fns.h"
#include "./hardware_fns/hardware_config.h"
#include "./uRos_fns/uRos_fns.h"
#include "./SteeringBraking/Steering.h"
bool ackermann_recv;

PID_vals brake_vals = PID_vals();

bool ros_enabled = true;


CurrState curr_state = CurrState();


ackermann_msgs__msg__AckermannDrive msg_ackermann;

//rcl_node_t node;
//uRos_s testSetup_throttle = uRos_s();  // Initialize the object properly if needed.
uRos_s testSetup = uRos_s();  // Initialize the object properly if needed.


TaskHandle_t TaskCore0;//Core 0 used for microROS, and other communication and soft(er) realtime things
TaskHandle_t TaskCore2;//Core 0 used for microROS, and other communication and soft(er) realtime things
TaskHandle_t TaskCore1;//Core 1 used for signals, PID loops, Sensor reading, motor control. hard(er) realtime




void setup() {


  motor_controller_setup();
  USER_SERIAL.begin(115200);
  hardware_setup();


#ifdef TRANSPORT_WIFI
    // uRos_init_wireless_node_ackermann(&testSetup, &throttle_callback_ackermann, &msg_ackermann_sub,
    //                                 "BELL310", "376F57AF1739", "192.168.2.58", 8887, "micro_ros_arduino_wifi_node_car", "/driveData");
    uRos_init_wireless_node_ackermann(&testSetup, &throttle_callback_ackermann, &msg_ackermann_sub,
                                      NET_SSID, NET_PASS, uROS_PORT, "micro_ros_arduino_wifi_node_car", "/driveData");
#endif
#ifdef TRANSPORT_SERIAL
    while(uRos_init_serial_node_ackermann(&testSetup, &throttle_callback_ackermann, &msg_ackermann, "micro_ros_arduino_wifi_node_car", "/driveData") != 0) {
      USER_SERIAL.println("Failed to initialize serial node, retrying...");
    } 
    USER_SERIAL.println("Serial node initialized");
    
#endif
    digitalWrite(GREEN_LED_PIN, LOW);

    xTaskCreatePinnedToCore(
      microROS_Task_pub,          // Task function
      "microRos_Task",        // Name of task
      4096,           // Stack size in words
      NULL,           // Task input parameter
      1,              // Priority of the task
      &TaskCore2,     // Task handle
      0);


    xTaskCreatePinnedToCore(
     steeringPID_task,          // Task function
    "steeringPID",        // Name of task
    4096,           // Stack size in words
    NULL,           // Task input parameter
    1,              // Priority of the task
    &TaskCore0,     // Task handle
    1);
  
}

void loop() {

  //rclc_executor_spin_some(&(testSetup.executor), RCL_MS_TO_NS(800));
}

