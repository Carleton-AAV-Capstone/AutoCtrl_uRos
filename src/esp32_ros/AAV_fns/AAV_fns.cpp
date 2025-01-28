
#include "AAV_fns.h"
#include "../hardware_fns/hardware_fns.h"
#include "../SteeringBraking/MotorCtrl.h"
extern Adafruit_MCP4725 dac;
extern bool ackermann_recv;

extern int setpoint;
extern PID_vals brake_vals;
extern PID_vals steer_vals;


void throttle_callback_ackermann(const void * msgin){
  const ackermann_msgs__msg__AckermannDrive * msg = (const ackermann_msgs__msg__AckermannDrive *)msgin;

  //USER_SERIAL.println("recv");
  USER_SERIAL.print(readSwitch(USE_RC, false));
  digitalWrite(2, HIGH);
  if (msg != NULL || readSwitch(USE_RC, false)) {
  // if (msg != NULL) {
    
    uint16_t accelMag;
    bool dir;
   
    
    
    if(!readSwitch(USE_RC, false)){

      if(msg->acceleration > 0){
        digitalWrite(DIR_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        accelMag = (uint16_t) msg->acceleration;
        
        dir = true;
      }else{
        digitalWrite(DIR_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
        accelMag = (uint16_t) msg->acceleration * -1;
        dir = false;
      }
      USER_SERIAL.println(msg->steering_angle);
      
      steer_vals.setpoint = (int) map((long) msg->steering_angle,-2000, 2000, 100, 0);
    }else{
      steer_vals.setpoint = map((long) readChannel(STR_RC, -255, 255, 0),-3200, 3200, 0, 100);
      accelMag = readChannel(THR_RC, -2000, 2000, 0);
      if(!readSwitch(REV_EN,false)){
        digitalWrite(DIR_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
      } else {
        digitalWrite(DIR_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
      }
      USER_SERIAL.println("USING RC");
    }
    if(!dir){
      USER_SERIAL.print("Steering Angle: "); USER_SERIAL.print((int) setpoint);
      USER_SERIAL.print("   voltage level(drv): ");
      USER_SERIAL.print( (int) accelMag);
      USER_SERIAL.print("  dir: "); USER_SERIAL.println((int) dir);
      dac.setVoltage((uint16_t) accelMag, false);
      brake_vals.setpoint = 0;
    }
    else{
      dac.setVoltage((uint16_t) 0, false);
      brake_vals.setpoint = (int) map((long) accelMag,0, -2000, 0, 100) * -1;
    }
    digitalWrite(2,LOW);
  }

}





