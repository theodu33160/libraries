#ifndef ROBOT_H
#define ROBOT_H


#include "Arduino.h"
#include<DCMotor.h>
#include "PS2MouseEgab.h"
#include <PID_v1.h>

class Robot
{
public :
    //Constructor :
    Robot(DCMotor* roueGauche, DCMotor* roueDroite, float empattement, float rayonRoues);
//    Robot(DCMotor *roueGauche, DCMotor *roueDroite, float empattement, float rayonRoues, PS2MouseEgab* sourisGache, PS2MouseEgab* sourisDroite, float angleInterSouris);
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
    void setAngleSvrPtCorrecteur(float kp, float ki, float kd);
    void setIntegralSaturation(float sat);

    float getRapportAvancerTourner();
    void setRapportAvancerTourner(float r);
    float modulo180(float angle);
    void debug();

    void jeuLeds();
    void initLeds();
    void allumerLeds();
    void eteindreLeds();

    bool tirageTirette();
    void reglagePinceManuel();
    void sensMoteurPince(int sens);
    bool serrerPince();
    bool desserrerPince();
    void arreterPince();
    bool obstaclePince();
    void ignoreCapteurPince();
    bool attrapperPalet();

private :
    DCMotor* m_roueGauche;
    DCMotor* m_roueDroite;
    PS2MouseEgab* m_sourisGauche;
    PS2MouseEgab* m_sourisDroite;

    // Variables TIERETTE
    byte pinDemarrageOutput = 48;
    byte pinDemarrageInput = 49;

    //Variables PINCE
    byte pinOuverturePince = 45;
    byte pinSerragePince = 47;
    byte pinInterrupteurPince = 46;
    byte pinInputPince = 44;
    byte pinIN1Pince = 42;
    byte pinIN2Pince = 40;
    byte pinCapteurPince = 15;
    int m_seuilBlocagePinceBas = 110;
    int m_seuilBlocagePinceHaut = 400;
    unsigned int tempsDemarrageMoteur = 70;
    bool m_fermetureBloquee = false;
    bool m_ouvertureBloquee = false;
    unsigned long debutOuverturePince = 0;
    unsigned long debutFermeturePince = 0;

    // actuellement, les leds sont branché de 26 à 29 et de 35 à 38
    byte ledON = 26;
    byte ledJaune = 27;
    byte ledArretUrgence = 28;
    byte ledEtapeFinie = 29;

    byte ledEtape1 = 35;
    byte ledEtape2 = 36;
    byte ledEtape3 = 37;
    byte ledEtape4 = 38;


    float m_empattement;        //distance entre le centre de rotation du robot et une roue en mm !
    float m_rayon; //37.64         //rayon des roues en mm
    double m_angle; // = 0                        // Sotck l'angle du robot au cours du temps, il va quand mÃªme rester Ã  dÃ©finir un cotÃ© + et un -
    double m_posX; //
    double m_posY; // =
    double m_angleInterSouris;
    double m_encodeurPrecedentGauche;
    double m_encodeurPrecedentDroit;
    byte m_acceleration=5;


    //Variables pour les correcteurs d'asservissement
    byte m_periode = 5;
    int m_lastTime =0;

    double m_consigneAngle; // consigne pour le correcteur PID en angle
    double m_vitesseRotation; // sortie du PID pour régler l'angle absolu du robot
    float m_Kp = 0.55;
    float m_Ki = 0.8;
    float m_Kd = 0;
    bool m_distanceComputed;

    double m_consigneAngleSvrPt; // consigne pour le correcteur PID en angle d'avancerTourner
    double m_vitesseRotationSvrPt; // sortie du PID pour régler l'angle absolu du robot
    float m_KpSvrPt = 0.25;
    float m_KiSvrPt = 0.25;
    float m_KdSvrPt = 0;
    bool m_angleComputedSvrPt;
    bool m_modeSvrPt;

    double m_consigneDistance = 0; //consigne pour le PID en distance
    double m_distance;   // entrée du PID
    double m_vitesseMoyenne; // sotie du PID
    float m_Kpd = 0.2;
    float m_Kid = 0.2;
    float m_Kdd = 0;
    bool m_angleComputed;

    //variables pour le maniement séquentiel du robot
    unsigned int m_compteur = 0;
    int m_etape = 0; //permet de savoir où en est le robot dans la rÃ©alisation des taches
    bool m_setupAllerRetour = false;
    float xInit = m_posX;
    float yInit = m_posY;
    float angleInit = m_angle;

    PID correctionAngle = PID(&m_angle, &m_vitesseRotation, &m_consigneAngle, m_Kp, m_Ki, m_Kd, true);
    PID correctionDistance = PID(&m_distance, &m_vitesseMoyenne, &m_consigneDistance, m_Kpd, m_Kid, m_Kdd, false);
    PID correctionAngleSvrPt = PID(&m_angle, &m_vitesseRotationSvrPt, &m_consigneAngleSvrPt, m_KpSvrPt, m_KiSvrPt, m_KdSvrPt, true);
    float m_rapportAvancerTourner = 0*0.05;

};
#endif // ROBOT_H






















