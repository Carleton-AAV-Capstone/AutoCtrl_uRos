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
        Wire.write(value >> 2);          // D9-D2
        Wire.write((value & 0x03) << 6); // D1-D0 in bits 7-6
        if (Wire.endTransmission() != 0) {
            USER_SERIAL.println("ERROR: DAC did not acknowledge!");
        }
        
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
