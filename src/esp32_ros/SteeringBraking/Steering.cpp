#include "Steering.h"

// External PID values for steering control
extern PID_vals steer_vals;

// Task to control steering using a PID loop
void steeringPID_task(void* parameter) {
    // Initialize the wake time for periodic task execution
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = STR_PID_TASK_DELAY / portTICK_PERIOD_MS; // Task runs every 10 ms

    while (true) {
        // Get the current steering position from the sensor
        float steerPos = getA1_scaled(STEER_ID);

        // Calculate elapsed time since the last PID update
        unsigned long currentTime = millis();
        float elapsedTime = (currentTime - steer_vals.lastTime) / 1000.0; // Convert to seconds
        steer_vals.lastTime = currentTime;

        // Calculate the error between the setpoint and the current position
        float error = steer_vals.setpoint - steerPos;

        // Apply a deadzone to minimize unnecessary adjustments
        if (abs(error) < steer_vals.positionDeadzone) {
            error = 0;
        }

        // Constrain the error to the specified limits
        error = constrain(error, -steer_vals.errorLimit, steer_vals.errorLimit);

        // Update the integral term only if the error is non-zero
        if (error != 0) {
            steer_vals.intval += error * elapsedTime;
            steer_vals.intval = constrain(steer_vals.intval, -steer_vals.integralLimit, steer_vals.integralLimit);
        } else {
            steer_vals.intval = 0; // Reset integral if no error
        }

        // Compute the proportional term
        float propval = error * steer_vals.P;

        // Compute the integral term
        float ival = steer_vals.intval * steer_vals.I;

        // Compute the derivative term
        float dval = ((error - steer_vals.prev_error) / elapsedTime) * steer_vals.D;
        steer_vals.prev_error = error;

        // Calculate the total motor power (PID output)
        ival = constrain(ival, -steer_vals.integralLimit, steer_vals.integralLimit);
        steer_vals.real_power = propval + ival + dval;

        // Constrain the motor power to the allowed range
        steer_vals.power = constrain(steer_vals.real_power, -steer_vals.maxPower, steer_vals.maxPower);

        // Apply the motor power to the steering motor
        setMotorSpeed(-steer_vals.power, STEER_ID);

        // Debugging output
        // USER_SERIAL.println("STEERING\n-----------------------------");
        // USER_SERIAL.print("Pos:"); USER_SERIAL.print(steerPos);
        // USER_SERIAL.print(", PropVal:"); USER_SERIAL.print(propval);
        // USER_SERIAL.print(", IVal:"); USER_SERIAL.print(ival);
        // USER_SERIAL.print(", Error:"); USER_SERIAL.print(error);
        // USER_SERIAL.print(", Setpoint:"); USER_SERIAL.print(steer_vals.setpoint);
        // USER_SERIAL.print(", RealPower:"); USER_SERIAL.print(steer_vals.real_power);
        // USER_SERIAL.print(", Power:"); USER_SERIAL.println(steer_vals.power);
        // USER_SERIAL.println("____________________________");

        // Delay to maintain the task's periodic execution
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
