#include "hardware_config.h"
#include "./motor_ctrl/MotorCtrl.h"
#include <IBusBM.h>
// #include "hardware_fns.h"
// #include "../SteeringBraking/MotorCtrl.h"

#include <PCA95x5.h>

PCA9555 ioex;
hw_timer_t *timer = NULL;
volatile bool flag = false; // Flag to indicate the interrupt occurred

void IRAM_ATTR ctrlTimer() {
    flag = true; // Set flag inside ISR
}


IBusBM ibus;
void hardware_setup(){
    //start i2c
    
    

    while(!Wire.begin()){
        USER_SERIAL.println("I2C init failed");
        delay(100);
    }


    ioex.attach(Wire);
    ioex.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
    ioex.direction(PCA95x5::Direction::OUT_ALL);
    ioex.write(PCA95x5::Level::H_ALL);

    ibus.begin(IBUS_SERIAL, IBUSBM_NOTIMER, IBUS_RX, IBUS_TX);//Initialising IBUS pin

    
    bool begin = false;
    while(!begin){

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

    
}


// Function to read a channel and map its value to a given range
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  
  ibus.loop();

  uint16_t ch = ibus.readChannel(channelInput);
  USER_SERIAL.print("Channel ");
    USER_SERIAL.print(channelInput);
    USER_SERIAL.print(" value: ");
    USER_SERIAL.println(ch);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {

  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
    
  return (ch > 50);
}

// Function to configure a pin as output
void ext_pinMode(uint8_t pin, bool isOutput) {
    if(isOutput){
        ioex.direction(static_cast<PCA95x5::Port::Port>(pin), PCA95x5::Direction::OUT);
    }else{
        ioex.direction(static_cast<PCA95x5::Port::Port>(pin), PCA95x5::Direction::IN);
    }
}

// Function to write a digital value to a pin
void ext_digitalWrite(uint8_t pin, bool value) {
    if(value){
        ioex.write(static_cast<PCA95x5::Port::Port>(pin), PCA95x5::Level::H);
    }else{
        ioex.write(static_cast<PCA95x5::Port::Port>(pin), PCA95x5::Level::L);
    }

}