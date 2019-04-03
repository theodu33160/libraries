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


byte etape =0;
byte compteur =0;


#include <Robot.h>
#include <DCMotor.h>
byte codeurGauche1 = 20; // pour interruptPin seulement les pin 2, 3, 18, 19, 20 , 21 fonctionnent
byte codeurGauche2 = 21; // pour interruptPin seulement les pin 2, 3, 18, 19, 20, 21 fonctionnent
byte codeurDroite1 = 19; // pour interruptPin seulement les pin 2, 3, 18, 19, 20, 21 fonctionnent
byte codeurDroite2 = 18; // pour interruptPin seulement les pin 2, 3, 18, 19, 20, 21 fonctionnent


DCMotor moteurDroit(2, 4, 3, codeurDroite1 , codeurDroite2);  //PWM, IN1, IN2, VOIEA, VOIEB
DCMotor moteurGauche(7,6,5, codeurGauche1 , codeurGauche2); //PWM, IN1, IN2, VOIEA, VOIEB

Robot monRobot = Robot(&moteurGauche,&moteurDroit, 207.5/2, 40); //roue gauche, roue droite, empattement, rayon roue


void setup() 
{ 
  TCCR3B = TCCR3B & 0b11111000 | 0x01;  //permet d'augmenter la fréquence du pwm des pins 5,3 et 2 au maximum (31 250Hz)
  TCCR4B = TCCR4B & 0b11111000 | 0x01;  //permet d'augmenter la fréquence du pwm des pins 8,7 et 6 au maximum (31 250Hz)
  
  Serial.begin(1000000);
  moteurDroit.setPID(1.429,-1.346,0);
  moteurGauche.setPID(0.6452,-0.5907,0);
/*  pinMode(codeurGauche1,INPUT);
  pinMode(codeurGauche2,INPUT);
  pinMode(codeurDroite1,INPUT);
  pinMode(codeurDroite2,INPUT);
  */
  attachInterrupt(digitalPinToInterrupt(codeurGauche1), motorCodeurIncrementalGA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(codeurGauche2), motorCodeurIncrementalGB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(codeurDroite1), motorCodeurIncrementalDA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(codeurDroite2), motorCodeurIncrementalDB, CHANGE);
  while(!monRobot.desserrerPince()) {}
  monRobot.initLeds();
  //delay(2000);
  //monRobot.jeuLeds();
}

void loop() 
{
    switch (etape)
    {
    case 0 :
        if (monRobot.suivrePoint(100,0,5))
        {
            compteur++;
            digitalWrite(ledEtape1,HIGH);
        }
        else
        {
            compteur = 0;
            digitalWrite(ledEtape1,LOW);
        }

        if (compteur>200)
        {
            compteur = 0;
            etape++;
        }
        break;
    case 1 :
        if (attrapperPalet())
        {
            compteur = 0;
            etape++;
        }
        break;
    case 2 :
        if (monRobot.suivrePoint(-100,0,10))
        {
            compteur++;
            digitalWrite(ledEtape3,HIGH);
        }
        else
        {
            compteur = 0;
            digitalWrite(ledEtape3,LOW);
        }
        if (compteur>10)
        {
            compteur = 0;
            etape++;
        }
        break;
    case 3 :
        if (monRobot.tournerPrecis(180,3))
        {
            compteur++;
            digitalWrite(ledEtape3,HIGH);
        }
        else
        {
            compteur = 0;
            digitalWrite(ledEtape3,LOW);
        }
        if (compteur>10)
        {
            compteur = 0;
            etape++;
        }

       break;
     case 4:
        if (monRobot.suivrePoint(-200,0,10))
        {
            compteur++;
            digitalWrite(ledEtape3,HIGH);
        }
        else
        {
            compteur = 0;
            digitalWrite(ledEtape3,LOW);
        }
        if (compteur>10)
        {
            compteur = 0;
            etape++;
        }
        break;
    case 5 :
        monRobot.suivrePoint(-200,0,10);
        monRobot.reglagePinceManuel();
    break;
    default :
       digitalWrite(ledJaune,HIGH);
    }
    monRobot.actualiserPosition();
    //monRobot.reglagePinceManuel();
    /*
    Serial.print("etape : ");
    Serial.print(etape);
    Serial.print("\tcompteur : ");
    Serial.println(compteur);
    */
/*
  monRobot.actualiserPosition();
  monRobot.suivrePoint(100,0,5);
  //moteurDroit.tourneRPM(10);
  //moteurGauche.tourneRPM(10);
  //monRobot.avancerTourner(10,0);
  //Serial.println(moteurGauche.getCodeur());
  //monRobot.debug();
*/
}

//10111213
bool attrapperPalet()
{
  digitalWrite(ledEtapeFinie,LOW);
  while (!monRobot.serrerPince()) {}
  unsigned long t0 = millis();
  while(millis()-t0<1000)
  {
    //monRobot.ignoreCapteurPince();
    monRobot.desserrerPince();
  }
  while (!monRobot.serrerPince()) {}
  digitalWrite(ledEtapeFinie,HIGH);
  return true;
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

