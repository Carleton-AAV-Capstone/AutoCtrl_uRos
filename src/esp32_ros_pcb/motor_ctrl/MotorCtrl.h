#include "../hardware_config.h"
#include <JrkG2.h>

#ifndef MCTRL_H
#define MCTRL_H



#define STEER_ID 0x01
#define BRAKE_ID 0x02

#define BRAKE_MAX 4095
#define DAC_ADDR_A0_FLOAT  0x48  // If A0 is floating

#define DAC_ADDR DAC_ADDR_A0_FLOAT



typedef struct pid_s{
    int setpoint;
    float intval;
    int maxPower;
    float real_power;
    float power;
    float prev_error;
    unsigned long lastTime;
    float P;
    float I;
    float D;
    int integralLimit;
    int positionDeadzone;
    int errorLimit;
}PID_vals;

void writeDAC(uint16_t value);

bool detectDAC();

#endif