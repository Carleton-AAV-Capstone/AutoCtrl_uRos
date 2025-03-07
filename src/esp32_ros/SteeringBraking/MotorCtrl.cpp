#include "MotorCtrl.h"
#include "esp_mac.h"


void writeDAC(uint16_t value) {
  value &= 0x3FF;  // Ensure value is 10-bit (0-1023)
  
  Wire.beginTransmission(DAC_ADDR);
  Wire.write((value >> 8) & 0xFF);  // Upper 8 bits
  Wire.write(value & 0xFF);         // Lower 8 bits
  if (Wire.endTransmission() != 0) {
      USER_SERIAL.println("ERROR: DAC did not acknowledge!");
  }
}

void detectDAC() {
  USER_SERIAL.println("Scanning for DAC...");
  Wire.beginTransmission(DAC_ADDR);
  if (Wire.endTransmission() == 0) {
      USER_SERIAL.println("DAC detected at 0x0C (A0 = GND)");
      return;
  }
  
  Wire.beginTransmission(DAC_ADDR);
  if (Wire.endTransmission() == 0) {
      USER_SERIAL.println("DAC detected at 0x46 (A0 = VCC)");
      return;
  }
  
  USER_SERIAL.println("ERROR: No DAC detected!");
}