#include <SoftwareSerial.h>

#define ACCZ_LSB 0x0C
#define ACCZ_MSB 0x0D
#define GYROZ_LSB 0x18
#define OPR_MODE 0x3D
#define ACCGYRO_MODE 0x15
#define ACC_MODE 0x11


SoftwareSerial bno(2, 3, false); //Rx ->Tx ,Tx ->Rx



char key;
unsigned char *incomingData = (char *) malloc(sizeof(char) * 4);
//unsigned char *regAddr = (char *) malloc(sizeof(char)*1);
unsigned char regAddr;
int i = 0;

void setup() {
  // ask a reading
  Serial.begin(115200);
  bno.begin(115200);
  emptyBno();
  Serial.println("Mise en Mode ");
  change_mode();    // mode accéléromètre + gyromètre
  Serial.println("End SETUP");
}

void loop() {
  delay(20);
  emptyBno();
  key = Serial.read();
  //Serial.print("on lit une accélération de\t");
  //Serial.println(getAcc());
  //  if (key == 'A') {   // The program enters here, but inside the function it cannot reada anything
  Serial.print("lecture acc : ");
  Serial.println(getValue(ACCZ_MSB) * 256 + getValue(ACCZ_LSB),DEC);
  Serial.println("");
  //  }
}

int getValue(byte addr) {
  int value;
  emptyBno();
  bno.write(0xAA); //start byte
  bno.write(0x1);  //read mode
  bno.write((byte) addr); //reg address
  bno.write(0x01);  //lengh
  delay(100);
  /*
    while (bno.available()) {
        incomingData[i] = bno.read();
        Serial.println(incomingData[i], HEX);
        i++;
      }
  */
  if (bno.read() != 0xBB) Serial.println("ERROR");
  if (bno.read() != 0x01) Serial.println("ERROR");
  value = bno.read();
  // Serial.println(value);

  return value;
}

void change_mode() {
  bno.write(0xAA);
  bno.write((byte)0x00); //ecriture
  bno.write(0x3D);
  bno.write(0x01);
  bno.write(ACCGYRO_MODE); // 11 = ACC ONLY ; 15 = ACCGYRO
}

void emptyBno()
{
  while (bno.available())
  {
    bno.read();
  }
}

