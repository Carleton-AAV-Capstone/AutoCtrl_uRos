#include "MotorCtrl.h"

int readByte()
{
  char c;
  CTRL_SERIAL.setTimeout(30);//set timeout for debugging if nothing is recieved
  if(CTRL_SERIAL.readBytes(&c, 1) == 0)
  {
    USER_SERIAL.println("READ_ERROR");
    return -1;  // Return -1 if no byte received
  }
  return (byte)c;  // Return the byte read
}


int sendCommand(uint8_t cmd, uint8_t deviceNum, uint16_t value){
    CTRL_SERIAL.write(0XAA);
    CTRL_SERIAL.write(deviceNum);
    // Send the speed (two bytes, split between 5 and 7 bits)
    CTRL_SERIAL.write(value & 0x1F);      // Send the least significant 5 bits
    CTRL_SERIAL.write((value >> 5) & 0x7F);  // Send the most significant 7 bits
}

// Function to exit safe mode and enable motor movement
// This must be called when the controller restarts or after any error
void exitSafeStart(int id)
{
  //CTRL_SERIAL.write(0x83);  // Command to exit safe start compact protocol
  CTRL_SERIAL.write(0xAA);
  CTRL_SERIAL.write(id);
}

// Function to set motor speed
// Speed should be in the range -3200 to 3200
void setMotorSpeed(int speed, int deviceNum)
{
  //USER_SERIAL.println("setMotorSpeed");
  // Handle negative speed (reverse)
  CTRL_SERIAL.write(0xAA);
  CTRL_SERIAL.write(deviceNum);
  if (speed < 0)
  {
    CTRL_SERIAL.write(0x06);  // Motor reverse command
    speed = -speed;       // Convert speed to positive
  }
  else
  {
    CTRL_SERIAL.write(0x05);  // Motor forward command
  }

  // Send the speed (two bytes, split between 5 and 7 bits)
  CTRL_SERIAL.write(speed & 0x1F);      // Send the least significant 5 bits
  CTRL_SERIAL.write((speed >> 5) & 0x7F);  // Send the most significant 7 bits
}

// Function to get a scaled sensor value from command A1
float getA1_scaled(int deviceNum) {
  CTRL_SERIAL.write(0xAA);  // Command A1
  CTRL_SERIAL.write(deviceNum);
  CTRL_SERIAL.write(0x21);    // Send the scaling factor
  if(deviceNum == STEER_ID){
    CTRL_SERIAL.write(14);
  }else if(deviceNum == BRAKE_ID){
    CTRL_SERIAL.write(18);
  }
  char b1 = readByte();     // Low byte
  char b2 = readByte();     // High byte (contains the sign)

//   USER_SERIAL.print(b1);
//   USER_SERIAL.print("  ");
//   USER_SERIAL.println(b2);

  // Combine bytes into a signed 16-bit integer
  int16_t number = (int16_t)((uint8_t)b1 | ((uint8_t)b2 << 8));
  if(number/32 > 110 || number/32 < -10){
    USER_SERIAL.println("A1_READ_ERROR");
  }
  // Scale and return the result
  return (float)number / 32.0;
}


void motor_controller_setup(){
  //  Initialize Serial2 with a baud rate of 9600 bps
  
  CTRL_SERIAL.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  // Initialize regular USER_SERIAL communication for debugging
 

  // Delay to allow motor controller to initialize (5 ms minimum)
  delay(5);
  
  // Send the baud rate auto-detect byte (0xAA) to the motor controller
  CTRL_SERIAL.write(0xAA);

  // Exit safe start to enable the motor
  exitSafeStart(STEER_ID);
  exitSafeStart(BRAKE_ID);
}