#include "Arduino.h"
#include "DCMotor.h"

DCMotor::DCMotor(byte brochePWM, byte IN1, byte IN2, byte voieA, byte voieB)
{
    m_broche = brochePWM;
    m_IN1 = IN1;
    m_IN2 = IN2;
    m_voieA = voieA;
    m_voieB = voieB;
    pinMode(m_broche,OUTPUT);
    pinMode(m_voieA,INPUT_PULLUP);
    pinMode(m_voieB,INPUT_PULLUP);
    pinMode(m_IN1,OUTPUT);
    pinMode(m_IN2,OUTPUT);
    digitalWrite(m_IN1, LOW);
    digitalWrite(m_IN2, LOW);
//    attachInterrupt(digitalPinToInterrupt(m_voieA), &this->codeurIncremental, CHANGE);
//    TCCR3B = TCCR3B & 0b11111000 | m_division_frequence;  //permet d'augmenter la fréquence du pwm de la pin 3 au maximum
//    TCCR4B = TCCR4B & 0b11111000 | m_division_frequence;  //permet d'augmenter la fréquence du pwm de la pin 4 au maximum
}

void DCMotor::actualiserVitesse()
{
    int ticks = m_compteur- m_precedentCompteur;
    m_precedentCompteur=m_compteur;
    m_vitesse = ticks * 60000 / (m_a *(m_periode));


}

void DCMotor::tourneRPM(float consigne)
{
    /*
    if (!Serial.available()) Serial.begin(1000000);
    Serial.print("\tC_mot ");
    Serial.print(consigne);
    Serial.print("\t");
    */
   // consigne = constrain(consigne,-1,165);    //le robot ne peut pas aller plus vite que ces valeurs
    int diff = millis() - m_temps_absolu;
    if (diff > 3*m_periode)  m_temps_absolu = millis() - m_periode;
    if (diff >=m_periode)
    {
        actualiserVitesse();
        m_error = consigne-m_vitesse;
        m_command = m_previousCommand + constrain(m_Kp * m_error + m_Ki * m_previousError,-acc/10,acc/10); //équation de récurrence PI
        int commandePuissance = constrain(int(m_command),-130,130);

        envoiCommande(commandePuissance);
        m_previousError=m_error;
        m_previousCommand=m_command;
        m_temps_absolu+=m_periode;
    }
}


void DCMotor::envoiCommande(int puissance)
{
    bool sens = puissance >= 0;

    if (puissance==0)
    {
        digitalWrite(m_IN1, LOW);
        digitalWrite(m_IN2, LOW);
    }
    else {
      puissance=abs(puissance)+90;  // le +90 est spécifique au pont en H utilisé et est dû à sa non linéarité
    }


    if (sens == 1){
      digitalWrite(m_IN1, HIGH);
      digitalWrite(m_IN2, LOW);
    }
    else {
      digitalWrite(m_IN1, LOW);
      digitalWrite(m_IN2, HIGH);
    }

    analogWrite(m_broche, puissance);
}

void DCMotor::codeurIncrementalA()
{
  int sens = 2*int(digitalRead(m_voieB)==digitalRead(m_voieA))-1; // renvoie 1 si on tourne dans le sens horaire, -1 sinon (ou inversement! mdr)
  m_compteur = m_compteur + sens;
}

void DCMotor::codeurIncrementalB()
{
  int sens = 2*int(digitalRead(m_voieA)!=digitalRead(m_voieB))-1; // renvoie 1 si on tourne dans le sens horaire, -1 sinon (ou inversement! mdr)
  m_compteur = m_compteur + sens;
}

void DCMotor::setPID(float kp, float ki, float kd)
{
    m_Kp = kp;
    m_Ki = ki;
    m_Kd = kd;
}

void DCMotor::setDivisionFrequence(byte facteur)
{
    m_division_frequence = facteur; // la fréquence vaut 62500Hz ou 31250Hz (fréquence maximale fournie par la PWM => provient de la fréquence du quartz / 256)
}

void DCMotor::setPeriode(int periode)
{
    m_periode = periode;
}

long DCMotor::getCodeur()
{
    return m_compteur;
}

float DCMotor::getVitesse()
{
    return m_vitesse;
}

float DCMotor::getReducteur()
{
    return m_a;
}

void DCMotor::setAcceleration(byte acc)
{
    m_acc = acc;
}
