#include "Arduino.h"


#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#define TRANSPORT_SERIAL
#undef TRANSPORT_WIFI

typedef struct curr_state_s {
    float speed;
    float steer_angle;
    float accel;
    bool dir;
}CurrState;

//rc pins
//22,0,34,35

//from motor controllers back: 35, 34, 0, 23: steering, throttle rev_en, RC_EN

//defined as channel numbers instead of pins with ibus configuration
#define USE_RC 4//channel 5 //make this a pulldown!!
#define REV_EN 5//channel 6
#define THR_RC 2 //channel 3
#define STR_RC 3 //channel 4

#define IBUS_SERIAL CTRL_SERIAL

#define DRIVER_ERROR_PIN 32

#define DIR_PIN 33
#define LED_PIN 2


#define I2C_SDA 23
#define I2C_SCL 22

#define BLUE_LED_PIN 14
#define RED_LED_PIN 18
#define GREEN_LED_PIN 19

#define ROS_SERIAL Serial1
#define USER_SERIAL Serial
#define CTRL_SERIAL Serial2

#define RX_PIN_1 RX1
#define TX_PIN_1 TX1

#define STEER_ID 0x5
#define BRAKE_ID 0x4




#define STEER_READ_MAX 2000//double check this
#define STEER_READ_MIN -2000
#define STEER_READ_DEFAULT 50


#define ACCEL_READ_MAX 2000
#define ACCEL_READ_MIN -2000
#define ACCEL_READ_DEFAULT 0

#define UROS_BAUD 115200
#define UROS_TX TX1
#define UROS_RX RX1

#define uROS_TASK_DELAY_SUB 10 // const TickType_t xFrequency = uROS_TASK_DELAY / portTICK_PERIOD_MS;
#define uROS_TASK_DELAY_PUB 100 // const TickType_t xFrequency = uROS_TASK_DELAY / portTICK_PERIOD_MS;
#define BRK_PID_TASK_DELAY 10 
#define STR_PID_TASK_DELAY 10 




//#define RX_PIN_1 3
//#define TX_PIN_1 1

#define TX_PIN 27
#define RX_PIN 26


void hardware_setup();
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue);
bool readSwitch(byte channelInput, bool defaultValue);
#endif