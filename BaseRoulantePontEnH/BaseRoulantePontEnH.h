#include<DCMotor.h>

#ifndef BASEROULANTEPONTENH_H
#define BASEROULANTEPONTENH_H


#include <PID_v1.h>
#include "Arduino.h"

class BaseRoulantePontEnH
{
public :
    //Constructor :
    BaseRoulantePontEnH(DCMotor* roueGauche, DCMotor* roueDroite, float empattement, float rayonRoues);
    void avancerTourner(int v, int theta);          // vitesseMoyenne, vitesseRotation
    void avancer(int vg, int vd); //vitesse roue gauche, vitesse roue droite
    bool tournerPrecis(float theta, float precision);       // angle, precision
    bool suivrePoint(float xCible, float yCible, float precision);   //x, y, precision
    void allerRetour(float x,float y); //x, y
    void actualiserPosition();
    void setPosition(double , double); //posX, posY
    void setPosition(double, double, double); //posX, posY et angle
    double getPosX();
    double getPosY();
    double getAngle();
    void setAcceleration(byte acc);
    void setAngleCorrecteur(float kp, float ki, float kd);       //P, I, D
    void setDistanceCorrecteur(float kp, float ki, float kd);    //P, I, D

    float getRapportAvancerTourner();
    void setRapportAvancerTourner(float r);
    int getBattery();
    float modulo180(float angle);
    void debug();

private :
    DCMotor* m_roueGauche;
    DCMotor* m_roueDroite;

    float m_empattement;        //distance entre le centre de rotation du robot et une roue en cm !
    float m_rayon;         //rayon des roues en mm
    double m_angle = 0; // = 0                        // Sotck l'angle du robot au cours du temps, il va quand mÃªme rester Ã  dÃ©finir un cotÃ© + et un -
    double m_posX = 0; //
    double m_posY = 0; // =
    double m_encodeurPrecedentGauche = 0;
    double m_encodeurPrecedentDroit = 0;
    byte m_acceleration=5;

    //Variables pour les correcteurs d'asservissement
    double m_consigneAngle; // consigne pour le correcteur PID en angle
    double m_vitesseRotation; // sortie du PID pour régler l'angle absolu du robot
    float m_Kp = 0.5; //70  //*PI/180
    float m_Ki = 1;  //1
    float m_Kd = 0;

    double m_consigneDistance = 0; //consigne pour le PID en distance
    double m_distance;   // entrée du PID
    double m_vitesseMoyenne; // sotie du PID
    float m_dKp = 2; //2.5
    float m_dKi = 1;
    float m_dKd = 0;

    //variables pour le maniement séquentiel du robot
    unsigned int m_compteur = 0;
    int m_etape = 1; //permet de savoir où en est le robot dans la rÃ©alisation des taches
    //int m_nbconf = 50; // le nombre de temps que le robot doit Ãªtre stable une fois la tache rÃ©ussi avant de passer Ã  la suivante

    PID correctionAngle = PID(&m_angle, &m_vitesseRotation, &m_consigneAngle, m_Kp, m_Ki, m_Kd);
    PID correctionDistance = PID(&m_distance, &m_vitesseMoyenne, &m_consigneDistance, m_dKp, m_dKi, m_dKd);
    float m_rapportAvancerTourner = 1;

};
#endif // BASEROULANTEPONTENH_H






















