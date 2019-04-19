#ifndef DCMOTOR_H
#define DCMOTOR_H

#include "Arduino.h"

class DCMotor
{
public:
    //constructeur
    DCMotor(byte brochePWM, byte IN1, byte IN2, byte voieA, byte voieB);

    // methodes
    void actualiserVitesse();
    void tourneRPM(float consigne);
    void envoiCommande(int puissance);

    void codeurIncrementalA();
    void codeurIncrementalB();

    long getCodeur();
    float getVitesse();
    float getReducteur();
    void setPID(float kp, float ki, float kd);
    void setDivisionFrequence(byte facteur);
    void setPeriode(int periode);

private:
    //-----------------DONNEES MESURE VITESSE-------------------
    byte m_voieA;   //broche de la voie A du codeur incrémental associé au moteur
    byte m_voieB;   //broche de la voie B du codeur incrémental associé au moteur
    unsigned long m_temps_absolu;
    int m_periode = 20; //période d'acquisition en ms
    volatile long m_compteur=0;
    long m_precedentCompteur=0;
    float m_vitesse=0;
    const float  m_a = 3200; //nombre de tic par tour de roue

    //-----------------DONNEES ASSERVISSEMENTS------------------------
    float m_error;
    float m_previousError=0;
    float m_command=0;
    float m_previousCommand=0;
    byte m_division_frequence = 0x01; // la fréquence vaut 62500Hz ou 31250Hz (fréquence maximale fournie par la PWM => provient de la fréquence du quartz / 256)

    //----PID----------PARAMETRES THEORIQUES----------------------------
    float m_Kp = 0.6452;
    float m_Ki = - 0.5907;
    float m_Kd = 0;

    //-------------------BRANCHEMENTS-----------------------
    byte m_broche = 2;   //ENA sur le pont en H
    byte m_IN1 = 3;
    byte m_IN2 = 4;
};

#endif // DCMOTOR_H
