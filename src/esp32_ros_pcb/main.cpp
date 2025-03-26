#include <JrkG2.h>
#include <Arduino.h>

#include "./uRos_fns/uRos_fns.h"

// #include "hardware_config.h"

bool ros_enabled = true;
volatile rmt_obj_t *rmtObj;
volatile uint32_t pulse_width = 0;    // Shared variable for pulse width
bool ackermann_recv;
CurrState curr_state = CurrState();

JrkG2I2C jrk_steer(STEER_ID);
JrkG2I2C jrk_brake(BRAKE_ID);


ackermann_msgs__msg__AckermannDrive msg_ackermann;


//rcl_node_t node;
//uRos_s testSetup_throttle = uRos_s();  // Initialize the object properly if needed.
uRos_s testSetup = uRos_s();  // Initialize the object properly if needed.


TaskHandle_t TaskCore0;//Core 0 used for microROS, and other communication and soft(er) realtime things
TaskHandle_t TaskCore2;//Core 0 used for microROS, and other communication and soft(er) realtime things
TaskHandle_t TaskCore1;//Core 1 used for signals, PID loops, Sensor reading, motor control. hard(er) realtime


// SemaphoreHandle_t i2cSemaphore;

void setup() {

  // i2cSemaphore = xSemaphoreCreateMutex();
  // if (i2cSemaphore == NULL) {
  //     USER_SERIAL.println("ERROR: Failed to create I2C semaphore");
  // }
  //motor_controller_setup();
  
  USER_SERIAL.begin(115200);
  hardware_setup();
  USER_SERIAL.println("hardware setup done");


#ifdef TRANSPORT_WIFI
    // uRos_init_wireless_node_ackermann(&testSetup, &throttle_callback_ackermann, &msg_ackermann_sub,
    //                                 "BELL310", "376F57AF1739", "192.168.2.58", 8887, "micro_ros_arduino_wifi_node_car", "/driveData");
    uRos_init_wireless_node_ackermann(&testSetup, &throttle_callback_ackermann, &msg_ackermann_sub,
                                      NET_SSID, NET_PASS, uROS_PORT, "micro_ros_arduino_wifi_node_car", "/driveData");
#endif
#ifdef TRANSPORT_SERIAL
      USER_SERIAL.println("Initializing serial node");
      //while(uRos_init_serial_node_ackermann(&testSetup, &throttle_callback_ackermann, &msg_ackermann, "micro_ros_arduino_wifi_node_car", "/carla/ego_vehicle/ackermann_control/control_info", "/carla/ego_vehicle/ackermann_cmd") != 0){
      while(uRos_init_serial_node_ackermann(&testSetup, &throttle_callback_ackermann, &msg_ackermann, "micro_ros_arduino_wifi_node_car", "/driveData", "/carla/ego_vehicle/ackermann_cmd") != 0){
      USER_SERIAL.println("Failed to initialize serial node");
    }
    USER_SERIAL.println("Serial node initialized");
    
#endif
    if(rmw_uros_sync_session(1000)){
      USER_SERIAL.println("Sync failed");
    }else{
      USER_SERIAL.println("Sync success");
    }
    //digitalWrite(GREEN_LED_PIN, LOW);

    xTaskCreatePinnedToCore(
      microROS_Task_pub,          // Task function
      "microRos_Task",        // Name of task
      4096,           // Stack size in words
      NULL,           // Task input parameter
      1,              // Priority of the task
      &TaskCore1,     // Task handle
      0);


    xTaskCreatePinnedToCore(
    microROS_Task_sub,          // Task function
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

