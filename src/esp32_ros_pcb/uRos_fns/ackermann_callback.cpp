#include "uRos_fns.h"
#include "../motor_ctrl/MotorCtrl.h"

extern JrkG2I2C jrk_steer;
extern JrkG2I2C jrk_brake;
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
          accelMag = (uint16_t) map(msg->acceleration, 0, 2000, 0, 1023);
          dir = true;
      } else {
          digitalWrite(DIR_PIN, HIGH);    
          accelMag = (uint16_t)(-msg->acceleration);
          dir = false;
      }
      
      // Map steering angle from input range to control range
      USER_SERIAL.print("steer_angle: ");
      USER_SERIAL.println(msg->steering_angle);
      jrk_steer.setTarget(map((long) msg->steering_angle, STEER_READ_MIN, STEER_READ_MAX, 0, 4095));
      // Apply control logic
      if (!dir) { // Moving forward
        jrk_brake.setTarget(0);
        USER_SERIAL.print("ACCEL_SETPOINT: ");
        USER_SERIAL.println(accelMag);
        writeDAC((uint16_t) accelMag);
        
      } else { // Applying braking force
        USER_SERIAL.print("BRAKING_SETPOINT: ");
        writeDAC((uint16_t) 0);
        jrk_brake.setTarget((int) map((long) accelMag, 0, ACCEL_READ_MIN, 0, BRAKE_MAX) * -1);
        //USER_SERIAL.println(brake_vals.setpoint);
        
      }
        
      
    }
    digitalWrite(LED_PIN, LOW);
}