#include "Arduino.h"
#include "DCMotor.h"

DCMotor::DCMotor(byte brochePWM, byte IN1, byte IN2, byte voieA, byte voieB)
{
    pinMode(brochePWM,OUTPUT);
    pinMode(voieA,INPUT_PULLUP);
    pinMode(voieB,INPUT_PULLUP);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    m_broche = brochePWM;
    m_IN1 = IN1;
    m_IN2 = IN2;
    m_voieA = voieA;
    m_voieB = voieB;
//    attachInterrupt(digitalPinToInterrupt(m_voieA), &this->codeurIncremental, CHANGE);
    TCCR3B = TCCR3B & 0b11111000 | m_division_frequence;  //permet d'augmenter la fréquence du pwm de la pin 3 au maximum
    TCCR4B = TCCR4B & 0b11111000 | m_division_frequence;  //permet d'augmenter la fréquence du pwm de la pin 4 au maximum
}

void DCMotor::actualiserVitesse()
{
    int ticks = m_compteur- m_precedentCompteur;
    m_precedentCompteur=m_compteur;
    m_vitesse = (float) (ticks) * 60000 / (m_a *(millis()-m_temps_absolu));

}

void DCMotor::tourneRPM(float consigne)
{
   // consigne = constrain(consigne,-1,165);    //le robot ne peut pas aller plus vite que ces valeurs
    if (millis() - m_temps_absolu >=m_periode)
    {
        actualiserVitesse();
        m_error = consigne-m_vitesse;
        m_command = m_previousCommand + m_Kp * m_error + m_Ki * m_previousError; //équation de récurrence PI
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
