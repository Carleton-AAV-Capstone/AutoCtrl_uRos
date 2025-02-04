#include "AAV_fns.h"
#include "../hardware_fns/hardware_fns.h"
#include "../SteeringBraking/MotorCtrl.h"
#include "../Control_Config.h"
// External global variables
extern Adafruit_MCP4725 dac;
extern bool ackermann_recv;
extern int setpoint;
extern PID_vals brake_vals;
extern PID_vals steer_vals;

/**
 * @brief Callback function for processing Ackermann drive messages.
 * 
 * This function reads the acceleration and steering angle values from the received 
 * Ackermann message and applies them to the control system. It also checks whether
 * the system should use remote control (RC) input instead.
 * 
 * @param msgin Pointer to the received Ackermann drive message.
 */
void throttle_callback_ackermann(const void * msgin) {
    const ackermann_msgs__msg__AckermannDrive * msg = (const ackermann_msgs__msg__AckermannDrive *)msgin;
    
    
    
    // Indicate message reception by setting pin 2 high
    digitalWrite(LED_PIN, HIGH);
    
    // Ensure the message is valid
    if (msg != NULL) {
      uint16_t accelMag;
      bool dir;

      // Process control input based on the source (message or RC input)      
      if (msg->acceleration > 0) {
          digitalWrite(DIR_PIN, LOW);          
          accelMag = (uint16_t) msg->acceleration;
          dir = true;
      } else {
          digitalWrite(DIR_PIN, HIGH);    
          accelMag = (uint16_t)(-msg->acceleration);
          dir = false;
      }
      
      // Map steering angle from input range to control range
      USER_SERIAL.print("steer_angle: ");
      USER_SERIAL.println(msg->steering_angle);
      steer_vals.setpoint = (int) map((long) msg->steering_angle, STEER_READ_MIN, STEER_READ_MAX, 100, 0);
      // Apply control logic
      if (!dir) { // Moving forward
        USER_SERIAL.print("ACCEL_SETPOINT: ");
        USER_SERIAL.println(accelMag);
        dac.setVoltage((uint16_t) accelMag, false);
        brake_vals.setpoint = 0;
      } else { // Applying braking force
        USER_SERIAL.print("BRAKING_SETPOINT: ");
        dac.setVoltage((uint16_t) 0, false);
        brake_vals.setpoint = (int) map((long) accelMag, 0, ACCEL_READ_MIN, 0, 100) * -1;
        USER_SERIAL.println(brake_vals.setpoint);
        
      }
        
      
    }
    digitalWrite(LED_PIN, LOW);
}


void RC_Control(){
  uint16_t accelMag;
  bool dir;
  steer_vals.setpoint = (long) readChannel(STR_RC, STEER_READ_MIN, STEER_READ_MAX, STEER_READ_DEFAULT);
  USER_SERIAL.print("steer_rc: ");
  USER_SERIAL.println(steer_vals.setpoint); 
  int accelMag_signed =  readChannel(THR_RC, ACCEL_READ_MIN, ACCEL_READ_MAX, ACCEL_READ_DEFAULT);
  digitalWrite(LED_PIN, HIGH);

  if (accelMag_signed > 0) {
      digitalWrite(DIR_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      accelMag = (uint16_t) accelMag_signed;
      dir = true;
  } else {
      accelMag = (uint16_t)(-accelMag_signed);
      dir = false;
  }

  // Reverse enable check
  if (readSwitch(REV_EN, false)) {
      digitalWrite(DIR_PIN, LOW);
  } else {
      digitalWrite(DIR_PIN, HIGH);
  }

  // Debugging output
  USER_SERIAL.print("ACCEL MAG: ");
  USER_SERIAL.println(accelMag);
  if (!dir) { // Moving forward
    USER_SERIAL.println("FORWARD");
    dac.setVoltage((uint16_t) accelMag, false);
    brake_vals.setpoint = 0;
  } else { // Applying braking force
    USER_SERIAL.print("I2C_status");    
    USER_SERIAL.println(dac.setVoltage((uint16_t) 0, false));
    USER_SERIAL.print("BRAKING_SETPOINT: ");
    
    brake_vals.setpoint = (int) map((long) accelMag, 0, ACCEL_READ_MIN, 0, 100) * -1;
    USER_SERIAL.println(brake_vals.setpoint);
  }
  digitalWrite(LED_PIN, LOW);
}