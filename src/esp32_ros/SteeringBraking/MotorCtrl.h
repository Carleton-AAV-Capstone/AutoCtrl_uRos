#include "../hardware_fns/hardware_config.h"
#ifndef MCTRL_H
#define MCTRL_H



#define STEER_ID 0x01
#define BRAKE_ID 0x02






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



int readByte();

void exitSafeStart(int deviceNum);

void setMotorSpeed(int speed, int deviceNum);

float getA1_scaled(int deviceNum);
float getA2_scaled(int deviceNum);

void motor_controller_setup();

#endif