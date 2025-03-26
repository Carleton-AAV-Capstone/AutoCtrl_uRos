#include "hardware_config.h"
#include <driver/rmt.h>
#include "./motor_ctrl/MotorCtrl.h"
// #include "hardware_fns.h"
// #include "../SteeringBraking/MotorCtrl.h"
extern rmt_obj_s *rmtObj;
void hardware_setup(){
    Wire.begin();

    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);

    digitalWrite(BLUE_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);

    pinMode(USE_RC, INPUT);
    pinMode(REV_EN, INPUT);
    pinMode(THR_RC, INPUT);
    pinMode(STR_RC, INPUT);

    pinMode(LED_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);


    rmtObj = rmtInit(USE_RC, false, RMT_MEM_64);

    bool begin = false;
    while(!begin){
        digitalWrite(RED_LED_PIN, LOW);
        delay(100);
        USER_SERIAL.println("CONNECTING TO DAC i2C");
        begin = detectDAC();
        if(!begin){
            USER_SERIAL.println("DAC CONNECTION FAIL");
        }

        }
  
    writeDAC(0);
    delay(250);
    digitalWrite(RED_LED_PIN, HIGH);
    delay(250);
    

    USER_SERIAL.println("DAC CONNECTION SUCCESS");
    //ROS_SERIAL.begin(115200, SERIAL_8N1, RX_PIN_1, TX_PIN_1);
    
}





// Function to read a channel and map its value to a given range
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
    //int ch = pulseIn(channelInput, HIGH, 1000); // Read PWM signal
    //rmtReadAsync(channelInput, 1000);
    uint32_t time = 0;
    rmt_data_t data;
    rmtReadAsync(rmtObj, &data, RMT_MEM_64, NULL, false, 10000);
    if(data.level0){
        time = data.duration0;
    }else{
        time = data.duration1;
    }
    USER_SERIAL.print("Time: ");
    USER_SERIAL.println(time);
    if (time < RC_MAX) return defaultValue; // Default value if signal is invalid
    return map(time, RC_INPUT_MIN_MAP, RC_INPUT_MAX_MAP, minLimit, maxLimit);
}



// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, RC_SW_MIN_LIM, RC_SW_MAX_LIM, intDefaultValue);
  return (ch > 50);
}