#include <Wire.h>

#define CMD                 (byte)0x00    // Values of 0 eing sent using write have to be cast as a byte to stop them being misinterperted as NULL
// This is a but with arduino 1
#define MD25ADDRESS         0x58          // Address of the MD25
#define SPEED1              (byte)0x00    // Byte to send speed to first motor
#define SPEED2              0x01          // Byte to send speed to second motor
#define encodeurONE         0x02          // Byte to read motor encodeur 1
#define encodeurTWO         0x06          // Byte to read motor encodeur 2
#define VOLTREAD            0x0A          // Byte to read battery volts
#define RESETencodeurs      0x20          // reset both encoders to 0
#define MODE                0x0E          // to change the mode

#define LCD_RX              0x02                              // RX and TX pins used for LCD0303 serial port
#define LCD_TX              0x03
#define LCD03_HIDE_CUR      0x04
#define LCD03_CLEAR         0x0C
#define LCD03_SET_CUR       0x02


void setup() {
  Wire.begin();
  // put your setup code here, to run once:
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);     //!\\  ces bits sont Ã©crits en dÃ©cimal
  Wire.write(0x31);
  Wire.write(125); 
  Wire.endTransmission();

}

void loop() {
  // put your main code here, to run repeatedly:

}
