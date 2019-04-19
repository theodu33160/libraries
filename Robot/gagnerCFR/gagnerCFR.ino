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


int etape =1;
byte compteur =0;
byte etapeBis = 0;
byte compteurBis = 0;

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
  monRobot.initLedsNB();
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

  monRobot.initCote();
  while(!monRobot.initLedsNB())
  {
    monRobot.reglagePinceManuel();
  }
  digitalWrite(ledON,HIGH);
}

void loop() 
{
  monRobot.allumerLedEtape(etapeBis);
  switch (etape)
  {
  case 1 :
      if (!monRobot.tirageTirette())
      {
        monRobot.reglagePinceManuel();
      }
      else compteur++;
      if (compteur>200)
      {
          monRobot.eteindreLeds();
          digitalWrite(ledON,HIGH);
          compteur = 0;
          etape++;
      }
      break;
  case 2 :
      if (deverrouillerGoldenium())
      {
        etape++;
        monRobot.jeuLeds();
        digitalWrite(ledON,HIGH);
      }
      break;
  case 3 :
      if (recupererGoldonium())
      {
          etape++;
          monRobot.jeuLeds();
          digitalWrite(ledON,HIGH);
      }
      break;
 case 4 :
      if (mettreGoldoniumDansBalance())
      {
          etape++;
          monRobot.jeuLeds();
          digitalWrite(ledON,HIGH);
      }
      break;
  default :
     monRobot.jeuLeds();
     delay(2000);
     break;
  }

  
  monRobot.actualiserPosition();
  //monRobot.reglagePinceManuel();
  
  //monRobot.debug();

   
  Serial.print("#");
  Serial.print(etape);
  Serial.print(" ");
  Serial.print(etapeBis);
  Serial.print("\t");
  Serial.println();
  /*
  Serial.print("\tcompteurBis : ");
  Serial.println(compteurBis);
  */
}

bool deverrouillerGoldenium()
{
  switch (etapeBis)
    {
    case 0 :
      if(monRobot.suivrePoint(1635,600,5))
      {
        compteurBis++;
        
      }
      else
      {
        compteurBis=0;
      }
      if(compteurBis>100)
      {
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis=0;
        etapeBis++;
      }
      break;

    case 1 :
      if(monRobot.tournerPrecis(-90,1))
      {
        compteurBis++;
      }
      else
      {
        compteurBis=0;
      }
      if(compteurBis>100){
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis=0;
        etapeBis++;
      }
    break;
    
    case 2 :
      if(monRobot.suivrePoint(1635,270,5))
      {
        compteurBis++;
      }
      else{
        compteurBis=0;
      }
      if(compteurBis>100){
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis=0;
        etapeBis++;
      }
    break;

    case 3 :
      if(monRobot.desserrerPince())
      {
        compteurBis++;
      }
      else{
        compteurBis=0;
      }
      if(compteurBis>10){
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis=0;
        etapeBis=0;
        return true ; //on passe à l'action suivante eheheh B) :D 
      }
    break;
    default :
      monRobot.avancer(0,0);
      digitalWrite(ledJaune,HIGH);
      monRobot.reglagePinceManuel();
    }
  return false;
}

bool recupererGoldonium()
{
  switch (etapeBis)
    { 
    case 0 :
      if(monRobot.suivrePoint(1635,400,10))
      {
        monRobot.eteindreLedEtape(etapeBis);
        etapeBis++;
      }
      break;
    case 1 :
      if(monRobot.tournerPrecis(5,5)){
        monRobot.eteindreLedEtape(etapeBis);
        etapeBis++;
      }
    break;
    
    case 2 :
      if(monRobot.suivrePoint(2224,600,5))
      {
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis++;   
      }
      else
      {
        compteurBis=0;
      }
      if(compteurBis>100)
      {
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis=0;
        etapeBis++;
      }
      break;

    case 3 :
      if(monRobot.tournerPrecis(-90,1)){
        compteurBis++;
      }
      else
      {
        compteurBis=0;
      }
      if(compteurBis>100){
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis=0;
        etapeBis++;
      }
    break;
    
    case 4 :
      if(monRobot.suivrePoint(2224,180,3)){
        compteurBis++;
      }
      else{
        compteurBis=0;
      }
      if(compteurBis>100){
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis=0;
        etapeBis++;
      }
    break;

    case 5 :
      if(monRobot.attraperPalet()){
        monRobot.eteindreLedEtape(etapeBis);
        etapeBis++;
      }
    break;

    case 6 :
      if(monRobot.suivrePoint(2224,300,10)){
        compteurBis++;
      }
      else{
        compteurBis=0;
      }
      if(compteurBis>100){
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis=0;

      }
    break;

    case 7 :
      if(monRobot.tournerPrecis(135,5)){
        monRobot.eteindreLedEtape(etapeBis);
        etapeBis=0;
        return true ;//on passe à l'action suivante eheheh B) :D 
      }
    break;

    default :
      monRobot.avancer(0,0);
      digitalWrite(ledJaune,HIGH);
      monRobot.reglagePinceManuel();
  }
  return false;
}


bool mettreGoldoniumDansBalance()
{
    switch (etapeBis)
    {
    case 0 :
      if(monRobot.suivrePoint(1300,1350,5))
      {
        compteurBis++;
        
      }
      else
      {
        compteurBis=0;
      }
      if(compteurBis>100)
      {
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis=0;
        etapeBis++;
      }
      break;

    case 1 :
      if(monRobot.tournerPrecis(-90,1))
      {
        compteurBis++;
      }
      else
      {
        compteurBis=0;
      }
      if(compteurBis>100){
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis=0;
        etapeBis++;
      }
    break;
    
    case 2 :
      if(monRobot.suivrePoint(1300,1500,5))
      {
        compteurBis++;
      }
      else{
        compteurBis=0;
      }
      if(compteurBis>100){
        monRobot.eteindreLedEtape(etapeBis);
        compteurBis=0;
        etapeBis++;
      }
    break;

    case 3 :
      if(monRobot.desserrerPince())
      {
        monRobot.eteindreLedEtape(etapeBis);
        etapeBis=0;
        return true ; //on passe à l'action suivante eheheh B) :D 
      }
    break;
    default :
      monRobot.avancer(0,0);
      digitalWrite(ledJaune,HIGH);
      monRobot.reglagePinceManuel();
    }
  return false;

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

