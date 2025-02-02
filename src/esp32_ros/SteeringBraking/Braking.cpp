#include "Braking.h"

extern PID_vals brake_vals;

/**
 * @brief Task to control braking using a PID loop.
 * 
 * This FreeRTOS task runs periodically, using a PID controller to adjust
 * braking force based on sensor feedback. The task calculates proportional,
 * integral, and derivative terms to determine the appropriate motor command.
 */
void brakingPID_task(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = BRK_PID_TASK_DELAY / portTICK_PERIOD_MS; 

    while (true) {
        // Read current brake position from the sensor
        float brakePos = getA2_scaled(BRAKE_ID);
        
        // Get the current time and calculate elapsed time (in seconds)
        unsigned long currentTime = millis();
        float elapsedTime = (currentTime - brake_vals.lastTime) / 1000.0;
        brake_vals.lastTime = currentTime;

        // Compute the error between setpoint and measured brake position
        float error = brake_vals.setpoint - brakePos;

        // Apply deadzone filtering to ignore small errors
        if (abs(error) < brake_vals.positionDeadzone) {
            error = 0;
        }

        // Limit the error to prevent excessive control efforts
        error = constrain(error, -brake_vals.errorLimit, brake_vals.errorLimit);

        // Update the integral term (accumulates error over time)
        if (error != 0) {
            brake_vals.intval += error * elapsedTime;
            brake_vals.intval = constrain(brake_vals.intval, -brake_vals.integralLimit, brake_vals.integralLimit);
        } else {
            brake_vals.intval = 0; // Reset integral term when error is negligible
        }

        // Compute PID control terms
        float propval = error * brake_vals.P;  // Proportional term
        float ival = brake_vals.intval * brake_vals.I;  // Integral term
        float dval = ((error - brake_vals.prev_error) / elapsedTime) * brake_vals.D;  // Derivative term
        brake_vals.prev_error = error;

        // Compute the total power output
        ival = constrain(ival, -brake_vals.integralLimit, brake_vals.integralLimit);
        brake_vals.real_power = propval + ival + dval;

        // Constrain motor power within operational limits
        brake_vals.power = constrain(brake_vals.real_power, -brake_vals.maxPower, brake_vals.maxPower);

        // Set the motor speed based on the computed power
        setMotorSpeed(brake_vals.power, BRAKE_ID);

        // Debugging output (commented out)
        /*
        USER_SERIAL.println("BRAKING\n-----------------------------");
        USER_SERIAL.print("Pos:"); USER_SERIAL.print(brakePos);
        USER_SERIAL.print(", PropVal:"); USER_SERIAL.print(propval);
        USER_SERIAL.print(", IVal:"); USER_SERIAL.print(ival);
        USER_SERIAL.print(", Error:"); USER_SERIAL.print(error);
        USER_SERIAL.print(", Setpoint:"); USER_SERIAL.print(brake_vals.setpoint);
        USER_SERIAL.print(", RealPower:"); USER_SERIAL.print(brake_vals.real_power);
        USER_SERIAL.print(", Power:"); USER_SERIAL.println(brake_vals.power);
        USER_SERIAL.println("____________________________");
        */

        // Wait until the next scheduled execution cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}