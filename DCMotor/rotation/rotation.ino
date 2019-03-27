#include <DCMotor.h>

const byte voieDroiteA = 19;
const byte voieDroiteB = 18;
const byte voieGaucheA = 20;
const byte voieGaucheB = 21;
DCMotor moteurDroite(2,4,3,voieDroiteA,voieDroiteB);
DCMotor moteurGauche(7,6,5,voieGaucheA,voieGaucheB); //PWM, IN1, IN2, VOIEA, VOIEB

void setup() 
{
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  //permet d'augmenter la fréquence du pwm de la pin 3 au maximum
  TCCR4B = TCCR4B & 0b11111000 | 0x01;  //permet d'augmenter la fréquence du pwm de la pin 4 au maximum
  Serial.begin(500000);
  //pinMode(voieA, INPUT_PULLUP);
  //pinMode(voieB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(voieDroiteA), motorCodeurIncrementalDA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(voieDroiteB), motorCodeurIncrementalDB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(voieGaucheA), motorCodeurIncrementalGA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(voieGaucheB), motorCodeurIncrementalGB, CHANGE);
  moteurDroite.setPID(1.429,-1.346,0);
  moteurGauche.setPID(0.6452,-0.5907,0);
}

void loop() 
{
  moteurDroite.tourneRPM(6);
  
  Serial.println(moteurDroite.getVitesse());
  Serial.print('\t');
  moteurGauche.tourneRPM(6);
  Serial.println(moteurGauche.getVitesse());
}

void motorCodeurIncrementalDA()
{
  moteurDroite.codeurIncrementalA();
}

void motorCodeurIncrementalDB()
{
  moteurDroite.codeurIncrementalB();
}

void motorCodeurIncrementalGA()
{
  moteurGauche.codeurIncrementalA();
}

void motorCodeurIncrementalGB()
{
  moteurGauche.codeurIncrementalB();
}

