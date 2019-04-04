//Type de carte de puissace pour les moteurs :
#define PONT_EN_H  //  PONT_EN_H - MD25

//Type de moyen de repérage du robot
#define CODEURS   // CODEURS    -   SOURIS    -   EXTERNE


byte ledON = 26;
byte ledJaune = 27;
byte ledArretUrgence = 28;
byte ledEtapeFinie = 29;

byte ledEtape1 = 35;
byte ledEtape2 = 36;
byte ledEtape3 = 37;
byte ledEtape4 = 38;


#include <Robot.h>
byte codeurGauche1 = 20; // pour interruptPin seulement les pin 2, 3, 18, 19, 20 , 21 fonctionnent
byte codeurGauche2 = 21; // pour interruptPin seulement les pin 2, 3, 18, 19, 20, 21 fonctionnent
byte codeurDroite1 = 19; // pour interruptPin seulement les pin 2, 3, 18, 19, 20, 21 fonctionnent
byte codeurDroite2 = 18; // pour interruptPin seulement les pin 2, 3, 18, 19, 20, 21 fonctionnent


DCMotor moteurDroit(2, 4, 3, codeurDroite1 , codeurDroite2);  //PWM, IN1, IN2, VOIEA, VOIEB
DCMotor moteurGauche(7,6,5, codeurGauche1 , codeurGauche2); //PWM, IN1, IN2, VOIEA, VOIEB

Robot monRobot = Robot(&moteurGauche,&moteurDroit, 207.5/2, 39.4); //roue gauche, roue droite, empattement = 207.5, rayon roue = 39.4


void setup() 
{ 
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  //permet d'augmenter la fréquence du pwm des pins 5,3 et 2 au maximum (31 250Hz)
  TCCR4B = TCCR4B & 0b11111000 | 0x01;  //permet d'augmenter la fréquence du pwm des pins 8,7 et 6 au maximum (31 250Hz)
  
  moteurDroit.setPID(1.429,-1.346,0);
  moteurGauche.setPID(0.9013, -0.826,0);

  attachInterrupt(digitalPinToInterrupt(codeurGauche1), motorCodeurIncrementalGA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(codeurGauche2), motorCodeurIncrementalGB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(codeurDroite1), motorCodeurIncrementalDA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(codeurDroite2), motorCodeurIncrementalDB, CHANGE);
  monRobot.setAngleCorrecteur(0.55,0.8,0);
  monRobot.setDistanceCorrecteur(0.01,0.2,0);
  monRobot.setAngleSvrPtCorrecteur(0.25,0.25,0);
  monRobot.setRapportAvancerTourner(0);
  monRobot.initLeds();
  //delay(2000);
  //monRobot.jeuLeds();
  //monRobot.setPosition(300,0,180);
}

void loop() 
{
  //monRobot.reglagePinceManuel();
  monRobot.tirageTirette();
  monRobot.actualiserPosition();
  monRobot.allerRetour(500,0);
  //monRobot.suivrePoint(2500,0,5);
  //monRobot.tournerPrecis(180,0.2);
  //monRobot.debug();   //Avec un Baudrate sur le serial de 1000000
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

