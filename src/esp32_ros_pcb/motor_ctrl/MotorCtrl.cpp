#include "MotorCtrl.h"
#include "esp_mac.h"


#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"



//extern SemaphoreHandle_t i2cSemaphore;

void writeDAC(uint16_t value) {
    value &= 0x3FF;  // Ensure value is 10-bit (0-1023)

    //if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        Wire.beginTransmission(DAC_ADDR);
        Wire.write((value >> 8) & 0xFF);  // Upper 8 bits
        Wire.write(value & 0xFF);         // Lower 8 bits
        if (Wire.endTransmission() != 0) {
            USER_SERIAL.println("ERROR: DAC did not acknowledge!");
        }
       // xSemaphoreGive(i2cSemaphore);
     else {
        USER_SERIAL.println("ERROR: Failed to acquire I2C semaphore for writeDAC");
    }
}

void detectDAC() {
    USER_SERIAL.println("Scanning for DAC...");

    //if (xSemaphoreTake(i2cSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        Wire.beginTransmission(DAC_ADDR);
        if (Wire.endTransmission() == 0) {
            USER_SERIAL.println("DAC detected at 0x0C (A0 = GND)");
            //xSemaphoreGive(i2cSemaphore);
            return;
        }

        Wire.beginTransmission(DAC_ADDR);
        if (Wire.endTransmission() == 0) {
            USER_SERIAL.println("DAC detected at 0x46 (A0 = VCC)");
            //xSemaphoreGive(i2cSemaphore);
            return;
        }

        //xSemaphoreGive(i2cSemaphore);
    //}

    USER_SERIAL.println("ERROR: No DAC detected!");
}
