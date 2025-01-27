
#include "AAV_fns.h"
#include "../hardware_fns/hardware_fns.h"

extern Adafruit_MCP4725 dac;
extern bool ackermann_recv;

extern int setpoint;



void throttle_callback_ackermann(const void * msgin){
  const ackermann_msgs__msg__AckermannDrive * msg = (const ackermann_msgs__msg__AckermannDrive *)msgin;

  //USER_SERIAL.println("recv");
  //USER_SERIAL.print(readSwitch(USE_RC, false));
  digitalWrite(2, HIGH);
  //if (msg != NULL || readSwitch(USE_RC, false)) {
  if (msg != NULL) {
    
    uint16_t accelMag;
    bool dir;
   
    
    
    //if(!readSwitch(USE_RC, false)){

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
      setpoint = (int) map((long) msg->steering_angle,-2000, 2000, 100, 0);
    // }else{
    //   setpoint = map((long) readChannel(STR_RC, -255, 255, 0),-3200, 3200, 0, 100);
    //   accelMag = readChannel(THR_RC, -2000, 2000, 0);
    //   //USER_SERIAL.println("USING RC");
    // }
    
    USER_SERIAL.print("Steering Angle: "); USER_SERIAL.print((int) setpoint);
    USER_SERIAL.print("   voltage level(drv): ");
    USER_SERIAL.print( (int) accelMag);
    USER_SERIAL.print("  dir: "); USER_SERIAL.println((int) dir);
    dac.setVoltage((uint16_t) accelMag, false);

    digitalWrite(2,LOW);
  }

}





