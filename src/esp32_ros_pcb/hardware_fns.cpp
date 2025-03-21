#include "hardware_config.h"
#include "./motor_ctrl/MotorCtrl.h"
// #include "hardware_fns.h"
// #include "../SteeringBraking/MotorCtrl.h"

void hardware_setup(){
    Wire.begin();

    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);

    digitalWrite(BLUE_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);

    pinMode(USE_RC, INPUT_PULLUP);
    pinMode(REV_EN, INPUT);
    pinMode(THR_RC, INPUT);
    pinMode(STR_RC, INPUT);

    pinMode(LED_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

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
    int ch = pulseIn(channelInput, HIGH, 100); // Read PWM signal
    if (ch < 990) return defaultValue; // Default value if signal is invalid
    return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}