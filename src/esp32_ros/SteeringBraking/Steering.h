#include <Arduino.h>
#include "../hardware_fns/hardware_config.h"
#include "MotorCtrl.h"
#include "../Control_Config.h"

void steeringPID_task(void* parameter);

void motor_controller_setup();