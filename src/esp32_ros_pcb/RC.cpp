

#include "rc.h"

extern JrkG2I2C jrk_steer;
extern JrkG2I2C jrk_brake;
void RC_Control(){
    uint16_t accelMag;
    bool dir;
    jrk_steer.setTarget(readChannel(STR_RC, 4095, 0, STEER_READ_DEFAULT));
    USER_SERIAL.print("steer_rc: ");
    //USER_SERIAL.println(steer_vals.setpoint); 
    int accelMag_signed =  readChannel(THR_RC, ACCEL_READ_MIN, ACCEL_READ_MAX, ACCEL_READ_DEFAULT);
    
  
    if (accelMag_signed > 0) {
        //digitalWrite(DIR_PIN, LOW);
        
        accelMag = (uint16_t) accelMag_signed;
        dir = true;
    } else {
        accelMag = (uint16_t)(-accelMag_signed);
        dir = false;
    }
  
    // Reverse enable check
     if (readSwitch(REV_EN, false)) {
         ext_digitalWrite(DIR_PIN, LOW);
     } else {
         ext_digitalWrite(DIR_PIN, HIGH);
     }
  
    // Debugging output
    USER_SERIAL.print("ACCEL MAG: ");
    USER_SERIAL.println(accelMag);
    if (!dir) { // Moving forward
      USER_SERIAL.println("FORWARD");
      //accelMag = map(accelMag, 0, ACCEL_READ_MAX, 0, 500);
      writeDAC((uint16_t) accelMag);
      jrk_brake.setTarget(0);
    } else { // Applying braking force
          
      writeDAC((uint16_t) 0);
      USER_SERIAL.print("BRAKING_SETPOINT: ");
      
      jrk_brake.setTarget(map((long) accelMag, 0, ACCEL_READ_MIN, 0, BRAKE_MAX) * -1);
      //USER_SERIAL.println(brake_vals.setpoint);
    }
    digitalWrite(LED_PIN, LOW);
  }