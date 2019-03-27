#include <BaseRoulantePontEnH.h>
#include <DCMotor.h>

const byte voieA = 20;
const byte voieB = 21;
DCMotor moteurDroit(2, 3, 4,  voieA, voieB);
DCMotor moteurGauche(7,5,6,18,19); //PWM, IN1, IN2, VOIEA, VOIEB

BaseRoulantePontEnH monRobot = BaseRoulantePontEnH(&moteurGauche,&moteurDroit, 127.5, 49.5);


void setup() 
{ 
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  //permet d'augmenter la fréquence du pwm de la pin 3 au maximum
  TCCR4B = TCCR4B & 0b11111000 | 0x01;  //permet d'augmenter la fréquence du pwm de la pin 4 au maximum
  
/*  Serial.begin(115200);
  Serial.println("###################pute##################################");
//  Serial.println(monRobot.getBattery());
*/

  attachInterrupt(digitalPinToInterrupt(voieA), motorCodeurIncrementalDA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(voieB), motorCodeurIncrementalDB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), motorCodeurIncrementalGA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), motorCodeurIncrementalGB, CHANGE);
}

void loop() 
{
  monRobot.allerRetour(50,0);
}

void motorCodeurIncrementalDA()
{
  moteurDroit.codeurIncrementalA();
}

void motorCodeurIncrementalDB()
{
  moteurDroit.codeurIncrementalB();
}

void motorCodeurIncrementalGA()
{
  moteurGauche.codeurIncrementalA();
}

void motorCodeurIncrementalGB()
{
  moteurGauche.codeurIncrementalB();
}

