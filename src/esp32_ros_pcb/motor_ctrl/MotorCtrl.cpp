#include "MotorCtrl.h"
#include "esp_mac.h"


#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"



//extern SemaphoreHandle_t i2cSemaphore;

void writeDAC(uint16_t value) {
    USER_SERIAL.print("DAC input: ");
    USER_SERIAL.println(value);
    value &= 0x3FF;  // Ensure value is 10-bit (0-1023)
    USER_SERIAL.print("Writing DAC value: ");
    USER_SERIAL.println(value);
    //if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
    uint16_t twoByte = (value ) & 0x0FFF; // Bits D9–D2 in upper byte
    
    // uint8_t upperByte = value >> 4;
    // uint8_t lowerByte = (uint8_t) twoByte;
    uint8_t upperByte = (value >> 6) & 0x0F;  // Upper 4 bits of the 10-bit value
    uint8_t lowerByte = (value << 2) & 0xFC;  // Lower 6 bits left-aligned

    // Begin I2C transmission to the DAC (R/W bit = 0 implied by Wire library)
    Wire.beginTransmission(DAC_ADDR_A0_GND);

    // Send the upper data byte (D9–D2), DAC should ACK
    Wire.write(upperByte);

    // Send the lower data byte (D1–D0), DAC should ACK
    Wire.write(lowerByte);

    
    
    // End transmissionand check for errors
 
    if (Wire.endTransmission() != 0) {
        USER_SERIAL.println("ERROR: DAC did not acknowledge!");
    }
        delay(1);
       // xSemaphoreGive(i2cSemaphore);
     
}

bool detectDAC() {
    USER_SERIAL.println("Scanning for DAC...");

    //if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        Wire.beginTransmission(DAC_ADDR);
        if (Wire.endTransmission() == 0) {
            USER_SERIAL.println("DAC detected at 0x0C (A0 = GND)");
            //xSemaphoreGive(i2cSemaphore);
            return 1;
        }

        return 0;

        //xSemaphoreGive(i2cSemaphore);
    //}

    USER_SERIAL.println("ERROR: No DAC detected!");
}
