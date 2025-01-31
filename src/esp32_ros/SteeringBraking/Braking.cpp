#include "Braking.h"

extern PID_vals brake_vals;

// extern int setpoint;
// extern float intval;
// extern int maxPower;
// extern float real_power;
// extern float power;
// extern float prev_error;
// extern unsigned long lastTime;

// extern int P;
// extern int I;
// extern int D;
// extern int integralLimit;
// extern int positionDeadzone;
// extern int errorLimit;


void brakingPID_task(void* parameter) {
  //USER_SERIAL.println("startBrakeTask");
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency =  10 / portTICK_PERIOD_MS; 
  while (true) {
    //USER_SERIAL.println("runBrakeTask");

  float brakePos = getA2_scaled(BRAKE_ID);
  
  // Get the time elapsed
  unsigned long currentTime = millis();
  //Calculate the elapsed time
  float elapsedTime = (currentTime - brake_vals.lastTime) / 1000.0;
  brake_vals.lastTime = currentTime;
  

  float error =  brake_vals.setpoint - brakePos;

  if(abs(error) < brake_vals.positionDeadzone){
    error = 0;
  }

  error = constrain(error, -brake_vals.errorLimit, brake_vals.errorLimit);

  // Calculate the integral term (accumulates the error over time)
  if(error != 0){
    brake_vals.intval += error * elapsedTime;
    brake_vals.intval = constrain(brake_vals.intval, -brake_vals.integralLimit, brake_vals.integralLimit);
  } else {
    brake_vals.intval = 0;
  }

  // Calculate the proportional term
  float propval = (error * brake_vals.P);

  // Calculate the integral term
  float ival = (brake_vals.intval * brake_vals.I);

  // Calculate the differential term
  float dval = (((error - brake_vals.prev_error)/elapsedTime) * brake_vals.D);
  brake_vals.prev_error = error;

  // Calculate the motor power based on proportional and integral control
  //real_power = propval + ival + dval;
  ival = constrain(ival, -brake_vals.integralLimit, brake_vals.integralLimit);
  brake_vals.real_power = propval + ival + dval;

  brake_vals.power = brake_vals.real_power;
  // Constrain the power within the range of the 
  brake_vals.power = constrain(brake_vals.power, -brake_vals.maxPower, brake_vals.maxPower);
  // Set the motor speed based on the calculated power
  setMotorSpeed(brake_vals.power, BRAKE_ID);

  // Output the current sensor value
  USER_SERIAL.println("BRAKING\n-----------------------------");
  USER_SERIAL.print("Pos:"); USER_SERIAL.print(brakePos);
  USER_SERIAL.print(", PropVal:"); USER_SERIAL.print(propval);
  USER_SERIAL.print(", IVal:"); USER_SERIAL.print(ival);
  USER_SERIAL.print(", Error:"); USER_SERIAL.print(error);
  USER_SERIAL.print(", Setpoint:"); USER_SERIAL.print(brake_vals.setpoint);
  USER_SERIAL.print(", RealPower:"); USER_SERIAL.print(brake_vals.real_power);
  USER_SERIAL.print(", Power:"); USER_SERIAL.println(brake_vals.power);
  USER_SERIAL.println("____________________________");
  vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait until next cycle  
    }
}