#include "Arduino.h"
#include <BaseRoulante.h>
#include <Wire.h>

//Constructor :
BaseRoulante::BaseRoulante (float empattement, float rayonRoues) // : rajouter un héritage ...
{
    Serial.begin(115200);
    Wire.begin();
    Serial.print("battery level : ");
//    Serial.println(BaseRoulante::getBattery()/10);
    Wire.beginTransmission(MD25ADDRESS);
    Wire.endTransmission();
    m_empattement = empattement;
    m_rayon = rayonRoues;
/*    BaseRoulante::setMode(3); //mode avancerTourner
    BaseRoulante::resetEncoders();
    BaseRoulante::setAutoRegulation();
    BaseRoulante::setAutoStopMotors();
    BaseRoulante::setAcceleration(4);
    m_posX = 0;
    m_posY = 0;
    m_angle = 0;
    m_encodeurPrecedentGauche = BaseRoulante::getEncoder(LEFTENCODER); //normalement ils valent 0 au dÃ©but
    m_encodeurPrecedentDroit = BaseRoulante::getEncoder(RIGHTENCODER);
  */
}


void BaseRoulante::allerRetour(float x, float y)
{
    //BaseRoulante::setPosition(0,0,0);
    byte etape = 0;
    byte compteur = 0;
    switch (etape)
    {
    case 0 :
        if (BaseRoulante::suivrePoint(x,y,0.2)) compteur++;
        else compteur = 0;
        if (compteur>100)
        {
            compteur = 0;
            etape++;
        }
        break;
    case 1 :
        if (BaseRoulante::tournerPrecis(PI,PI/180)) compteur++;
        else compteur = 0;
        if (compteur > 100)
        {
            compteur = 0;
            etape++;
        }
        break;
    case 2 :
        if (BaseRoulante::suivrePoint(0,0,0.2)) compteur++;
        else compteur = 0;
        if (compteur>100)
        {
            compteur = 0;
            etape++;
        }
        break;
    case 3 :
        if (BaseRoulante::tournerPrecis(0,PI/180)) compteur++;
        else compteur = 0;
        if (compteur > 100)
        {
            compteur = 0;
            etape = 0;
        }
        break;
    default :
        etape = 0;
        break;
    }
    BaseRoulante::actualiserPosition();
}


void BaseRoulante::actualiserPosition() // tourne Ã  une visteese x1 pour le moteur 1 et idem pour le 2 : x1 = GAUCHE
{
  double encodeurGauche = BaseRoulante::getEncoder(LEFTENCODER); //prend 3 ms
  double encodeurDroit = BaseRoulante::getEncoder(RIGHTENCODER); // prend 3 ms
  int nbPasEncodeurGauche = encodeurGauche - m_encodeurPrecedentGauche;
  int nbPasEncodeurDroit = encodeurDroit - m_encodeurPrecedentDroit;
  m_encodeurPrecedentGauche = encodeurGauche;
  m_encodeurPrecedentDroit = encodeurDroit;

  //Pour évite les gros bugs non physiques
  if (abs(nbPasEncodeurGauche) + abs(nbPasEncodeurDroit) < 30)
  {
    //dans un intervalle de temps :
    double distance_parcourue_roue_gauche = nbPasEncodeurGauche * 2 * PI * m_rayon / 360; //il faut surement rajouter le rapport de rÃ©duction du moteur                       // ici on calcule la distance parcourue. Pour cela, on estime que le temps d'exÃ©cution de l'algo est faible, et le seul facteur limitant est
    double distance_parcourue_roue_droite = nbPasEncodeurDroit * 2 * PI * m_rayon / 360;
    double distance_parcourue = (distance_parcourue_roue_gauche + distance_parcourue_roue_droite) / 2;     // on constate avec un raisonmment physique que la distance parcourue par le ce,tre du robot est la moitiÃ© de la somme de celles parcourues par chacunes de ses roues
    m_angle = m_angle + atan((distance_parcourue_roue_gauche - distance_parcourue_roue_droite) / (2 * m_empattement));             // cf simple calme d'angles dans un triangle + ajout de la fonction arctan ðŸ™‚
    m_angle = moduloPI(m_angle); // permet d'avoir un angle entre -pi et +pi
    m_posX += distance_parcourue * cos(m_angle);
    m_posY += distance_parcourue * sin(m_angle);
  }

/*  if (debug)
  {
    Serial.print("\tposX :\t");
    Serial.print(m_posX);
    Serial.print("\tposY :\t");
    Serial.print(m_posY);
    Serial.print("\tangle ° :\t");
    Serial.print(180 * m_angle / PI);
    Serial.print("\ttemps\t");
    Serial.println(millis());
  } */
}


bool BaseRoulante::tournerPrecis(float theta, float precision) // le robot vise l'angle theta en radians
{
  m_consigneAngle = theta;
  correctionAngle.Compute();
/*  if (debug)
  {
    Serial.print("\tvitesse de rotation :\t");
    Serial.print(m_vitesseRotation);
  }
*/
  if (abs(theta - m_angle) < precision) {
    avancerTourner(0, 0);
    return true;
  }
  BaseRoulante::avancerTourner(0, m_vitesseRotation);
  return false;
}

bool BaseRoulante::suivrePoint(float xCible, float yCible, float precision)
{
  m_distance = sqrt(pow((m_posX - xCible), 2) + pow((m_posY - yCible), 2));
  int sens = ((cos(m_angle) * (xCible - m_posX) + sin(m_angle) * (yCible - m_posY) ) > 0 ) * 2 - 1; // produit scalaire pour savoir si le robot a dépasser la cible
  m_distance = - sens * m_distance;
/* if (debug)
{
  Serial.print("\tdistance");
  Serial.print(m_distance);
  Serial.print("\t");
} */

  correctionDistance.Compute();


  m_consigneAngle = atan((yCible - m_posY) / (xCible - m_posX));
  correctionAngle.Compute();

  BaseRoulante::avancerTourner(int(m_vitesseMoyenne / (1 + m_vitesseRotation)), m_vitesseRotation);

  if (abs(m_distance) < precision)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    BaseRoulante::avancerTourner(0, 0);
    return true;
  }
  digitalWrite(LED_BUILTIN, LOW);
  return false;
}



void BaseRoulante::avancerTourner(int v, int theta)
{
  theta = constrain(theta, -128, 127);
  v = constrain(v, -128 + abs(theta), 127 - abs(theta));
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED1);
  Wire.write(v);
  Wire.endTransmission();
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED2);
  Wire.write(theta);
  Wire.endTransmission();
}

int BaseRoulante::getBattery()
{
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(VOLTAGE_BATTERY);
  Wire.endTransmission();
  Wire.requestFrom(MD25ADDRESS, 3);
  return Wire.read();
}

void BaseRoulante::setMode(byte mode)
{
    Serial.print("coucou1");
  Wire.beginTransmission(MD25ADDRESS);
  Serial.print("coucou2");
  Wire.write(MODE);
  Serial.print("coucou3");
  Wire.write(mode); // Mode avancerTourner avec -128  (full reverse)  0 (stop)   127 (full forward)
  Serial.print("coucou4");
  Wire.endTransmission();
  Serial.print("coucou5");
}

void BaseRoulante::resetEncoders() {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(MORECMD);
  Wire.write(RESETENCODERS);
  Wire.endTransmission();
}

void BaseRoulante::setAutoRegulation()
{
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(MORECMD);
  Wire.write(AUTOREGULATION);
  Wire.endTransmission();
}

void BaseRoulante::setNoAutoStopMotors()
{
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(MORECMD);
  Wire.write(NOSTOP);
  Wire.endTransmission();
}

void BaseRoulante::setAutoStopMotors()
{
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(MORECMD);
  Wire.write(AUTOSTOPMOTOR);
  Wire.endTransmission();
}

void BaseRoulante::setAcceleration(byte acc)
{
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ACCELERATION);
  Wire.write(acc);
  Wire.endTransmission();
}

double BaseRoulante::getEncoder(byte encodeur)
{ // Function to read and display value of encodeur 1 as a long
  unsigned long temps0 = millis();
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(encodeur);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);                                                 // Request 4 bytes from MD25
  while (Wire.available() < 4);                                                     // Wait for 4 bytes to arrive
  {
    if (millis() - temps0 > 20) return 0;
  }
  long total = Wire.read();                                                       // First byte for encodeur 1, HH.
  total <<= 8;
  total += Wire.read();                                                           // Second byte for encodeur 1, HL
  total <<= 8;
  total += Wire.read();                                                          // Third byte for encodeur 1, LH
  total <<= 8;
  total  += Wire.read();   // Fourth byte for encodeur 1, LL
  return (total);
}

void BaseRoulante::setPosition(double x, double y, double angle)
{
    m_posX = x;
    m_posY = y;
    m_angle = angle;
}

double BaseRoulante::getPosX()
{
    return m_posX;
}

double BaseRoulante::getPosY()
{
    return m_posY;
}

double BaseRoulante::getAngle()
{
    return m_angle;
}

float BaseRoulante::moduloPI(float angle)
{
  return angle - 2 * PI * floor((angle + PI) / 2 / PI);
}






















