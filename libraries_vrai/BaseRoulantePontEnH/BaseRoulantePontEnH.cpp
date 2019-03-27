#include <BaseRoulantePontEnH.h>


//Constructor :
BaseRoulantePontEnH::BaseRoulantePontEnH(DCMotor* roueGauche, DCMotor* roueDroite, float empattement, float rayonRoues) // : rajouter un héritage ...  float empattement = 127.5, float rayonRoues = 49.5)
{
    Serial.begin(1000000);
    Serial.print("battery level : ");
//    Serial.println(BaseRoulantePontEnH::getBattery()/10);
    m_roueGauche = roueGauche;
    m_roueDroite = roueDroite;
    m_empattement = empattement;
    m_rayon = rayonRoues;
    m_encodeurPrecedentGauche = m_roueGauche->getCodeur(); //normalement ils valent 0 au dÃ©but
    m_encodeurPrecedentDroit = m_roueDroite->getCodeur();
}

void BaseRoulantePontEnH::debug()
{
    if (!Serial.available()) Serial.begin(1000000);
    Serial.print("temps\t");
    Serial.print(millis());
    Serial.print("\tposX (mm) :\t");
    Serial.print(m_posX);
    Serial.print("\tposY (mm) :\t");
    Serial.print(m_posY);
    Serial.print("\tangle ° :\t");
    Serial.print(m_angle);
    Serial.print("\tvitesse de rotation :\t");
    Serial.print(m_vitesseRotation);
    Serial.print("\tdistance :");
    Serial.print(m_distance);
    Serial.println("\t");
}

void BaseRoulantePontEnH::allerRetour(float x, float y)
{
    float xInit = m_posX;
    float yInit = m_posY;
    float angleInit = m_angle;
    byte etape = 0;
    byte compteur = 0;
    switch (etape)
    {
    case 0 :
        if (BaseRoulantePontEnH::suivrePoint(x,y,2))
        {
            compteur++;
        }
        else compteur = 0;
        if (compteur>200)
        {
            compteur = 0;
            etape++;
        }
        break;
    case 1 :
        if (BaseRoulantePontEnH::tournerPrecis(angleInit+180,1))
        {
            compteur++;
        }
        else compteur = 0;
        if (compteur > 200)
        {
            compteur = 0;
            etape++;
        }
        break;
    case 2 :
        if (BaseRoulantePontEnH::suivrePoint(xInit,yInit,2))
        {
            compteur++;
        }
        else compteur = 0;
        if (compteur>200)
        {
            compteur = 0;
            etape++;
        }
        break;
    case 3 :
        if (BaseRoulantePontEnH::tournerPrecis(angleInit,1))
        {
            compteur++;
        }
        else compteur = 0;
        if (compteur > 200)
        {
            compteur = 0;
            etape = 0;
        }
        break;
    case 4 :
        if (BaseRoulantePontEnH::suivrePoint(xInit,yInit,2))
        {
            compteur++;
        }
        else compteur = 0;
        if (compteur > 1000)
        {
            compteur = 0;
            etape = 0;
        }
        break;
    default :
        etape = 0;
        break;
    }
    BaseRoulantePontEnH::actualiserPosition();
}


void BaseRoulantePontEnH::actualiserPosition() // tourne Ã  une visteese x1 pour le moteur 1 et idem pour le 2 : x1 = GAUCHE
{
  double encodeurGauche = m_roueGauche->getCodeur();
  double encodeurDroit = m_roueGauche->getCodeur();
  int nbPasEncodeurGauche = encodeurGauche - m_encodeurPrecedentGauche;
  int nbPasEncodeurDroit = encodeurDroit - m_encodeurPrecedentDroit;
  m_encodeurPrecedentGauche = encodeurGauche;
  m_encodeurPrecedentDroit = encodeurDroit;

  //Pour évite les gros bugs non physiques
  if (abs(nbPasEncodeurGauche) + abs(nbPasEncodeurDroit) < 300)
  {
    //dans un intervalle de temps :
    double distance_parcourue_roue_gauche = nbPasEncodeurGauche * 2 * PI * m_rayon / m_roueGauche->getReducteur(); //il faut surement rajouter le rapport de rÃ©duction du moteur                       // ici on calcule la distance parcourue. Pour cela, on estime que le temps d'exÃ©cution de l'algo est faible, et le seul facteur limitant est
    double distance_parcourue_roue_droite = nbPasEncodeurDroit * 2 * PI * m_rayon / m_roueDroite->getReducteur();
    double distance_parcourue = (distance_parcourue_roue_gauche + distance_parcourue_roue_droite) / 2;     // on constate avec un raisonmment physique que la distance parcourue par le ce,tre du robot est la moitiÃ© de la somme de celles parcourues par chacunes de ses roues
    m_angle = m_angle + 180/PI*atan((distance_parcourue_roue_gauche - distance_parcourue_roue_droite) / (2 * m_empattement));             // cf simple calme d'angles dans un triangle + ajout de la fonction arctan ðŸ™‚
    m_angle = modulo180(m_angle); // permet d'avoir un angle entre -180 et +180°
    m_posX += distance_parcourue * cos(PI/180*m_angle);
    m_posY += distance_parcourue * sin(PI/180*m_angle);
  }
}


bool BaseRoulantePontEnH::tournerPrecis(float theta, float precision) // le robot vise l'angle theta en radians
{
  m_consigneAngle = theta;
  modulo180(theta);
  correctionAngle.Compute();
  if (abs(theta - m_angle) <= precision) {
    avancerTourner(0, 0);
    return true;
  }
  BaseRoulantePontEnH::avancerTourner(0, m_vitesseRotation);
  return false;
}

bool BaseRoulantePontEnH::suivrePoint(float xCible, float yCible, float precision)
{
  m_distance = sqrt(pow((m_posX - xCible), 2) + pow((m_posY - yCible), 2));
  int sens = ((cos(m_angle) * (xCible - m_posX) + sin(m_angle) * (yCible - m_posY) ) > 0 ) * 2 - 1; // produit scalaire pour savoir si le robot a dépasser la cible
  m_distance = sens * m_distance;
  correctionDistance.Compute();

  m_consigneAngle = atan((yCible - m_posY) / (xCible - m_posX));
  correctionAngle.Compute();

  BaseRoulantePontEnH::avancerTourner(int(m_vitesseMoyenne / (1 + m_vitesseRotation*m_rapportAvancerTourner)), m_vitesseRotation);

  if (abs(m_distance) <= precision)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    //BaseRoulantePontEnH::avancerTourner(0, 0);
    return true;
  }
  digitalWrite(LED_BUILTIN, LOW);
  return false;
}


void BaseRoulantePontEnH::avancerTourner(int v, int theta)
{
  theta = constrain(theta, -128, 127);
  v = constrain(v, -128 + abs(theta), 127 - abs(theta));
  avancer(v-theta,v+theta);
}

void BaseRoulantePontEnH::avancer(int vg, int vd)
{
    vg = constrain(vg,-128,127);
    vd = constrain(vd,-128,127);
    m_roueGauche->tourneRPM(vg);
    m_roueDroite->tourneRPM(vd);
}

int BaseRoulantePontEnH::getBattery()
{
  return 0;
}

void BaseRoulantePontEnH::setAcceleration(byte acc)
{
  m_acceleration = acc;
}

void BaseRoulantePontEnH::setPosition(double x, double y, double angle)
{
    m_posX = x;
    m_posY = y;
    m_angle = angle;
}

double BaseRoulantePontEnH::getPosX()
{
    return m_posX;
}

double BaseRoulantePontEnH::getPosY()
{
    return m_posY;
}

double BaseRoulantePontEnH::getAngle()
{
    return m_angle;
}

void BaseRoulantePontEnH::setRapportAvancerTourner(float r)
{
    m_rapportAvancerTourner = r;
}

float BaseRoulantePontEnH::getRapportAvancerTourner()
{
    return m_rapportAvancerTourner;
}

void BaseRoulantePontEnH::setAngleCorrecteur(float kp, float ki, float kd)
{
    m_Kp = kp;
    m_Ki = ki;
    m_Kd = kd;
}

void BaseRoulantePontEnH::setDistanceCorrecteur(float kp, float ki, float kd)
{
    m_dKp = kp;
    m_dKi = ki;
    m_dKd = kd;
}


float BaseRoulantePontEnH::modulo180(float angle)
{
  return angle - 360 * floor((angle + 180) / 360);
}
