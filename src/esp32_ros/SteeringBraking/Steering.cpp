

#include "Steering.h"


extern PID_vals steer_vals;




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


void steeringPID_task(void* parameter) {
  //USER_SERIAL.println("startBrakeTask");
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency =  10 / portTICK_PERIOD_MS; // 500 ms interval
  while (true) {
    //USER_SERIAL.println("runBrakeTask");

  float steerPos = getA1_scaled(STEER_ID);
  
  // Get the time elapsed
  unsigned long currentTime = millis();
  //Calculate the elapsed time
  float elapsedTime = (currentTime - steer_vals.lastTime) / 1000.0;
  steer_vals.lastTime = currentTime;
  

  float error =  steer_vals.setpoint - steerPos;

  if(abs(error) < steer_vals.positionDeadzone){
    error = 0;
  }

  error = constrain(error, -steer_vals.errorLimit, steer_vals.errorLimit);

  // Calculate the integral term (accumulates the error over time)
  if(error != 0){
    steer_vals.intval += error * elapsedTime;
    steer_vals.intval = constrain(steer_vals.intval, -steer_vals.integralLimit, steer_vals.integralLimit);
  } else {
    steer_vals.intval = 0;
  }

  // Calculate the proportional term
  float propval = (error * steer_vals.P);

  // Calculate the integral term
  float ival = (steer_vals.intval * steer_vals.I);

  // Calculate the differential term
  float dval = (((error - steer_vals.prev_error)/elapsedTime) * steer_vals.D);
  steer_vals.prev_error = error;

  // Calculate the motor power based on proportional and integral control
  //real_power = propval + ival + dval;
  ival = constrain(ival, -steer_vals.integralLimit, steer_vals.integralLimit);
  steer_vals.real_power = propval + ival + dval;

  steer_vals.power = steer_vals.real_power;
  // Constrain the power within the range of the 
  steer_vals.power = constrain(steer_vals.power, -steer_vals.maxPower, steer_vals.maxPower);
  // Set the motor speed based on the calculated power
  setMotorSpeed(-steer_vals.power, STEER_ID);

    USER_SERIAL.println("STEERING\n-----------------------------");
  // Output the current sensor value
  USER_SERIAL.print("Pos:");
  USER_SERIAL.print(steerPos);

  USER_SERIAL.print(",propval:");
  USER_SERIAL.print(propval);
  
  USER_SERIAL.print(",ival:");
  USER_SERIAL.print(ival);

  USER_SERIAL.print(",error:");
  USER_SERIAL.print(error);
  // Set point
  USER_SERIAL.print(",Setpoint:");
  USER_SERIAL.print(steer_vals.setpoint);
  // Print the real power value
  USER_SERIAL.print(",RealPower:");
  USER_SERIAL.print(steer_vals.real_power);
  // Output the calculated motor power
  USER_SERIAL.print(",Power:");
  USER_SERIAL.println(steer_vals.power);
  USER_SERIAL.println("____________________________");
  vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait until next cycle  
    }
}