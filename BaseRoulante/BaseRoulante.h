#ifndef BASEROULANTE_H
#define BASEROULANTE_H

#include <PID_v1.h>

#define CMD                 (byte)0x00
#define MD25ADDRESS         0x58
#define VOLTAGE_BATTERY     10
#define SPEED1              (byte) 0x00
#define SPEED2              0x01
#define MODE                15
#define MORECMD             16
#define AUTOREGULATION      0x31
#define NOSTOP              0x32 //don't stop the motors if no datas are sent to the card
#define AUTOSTOPMOTOR       0x33 // stop the motors if no datas are sent to the card
#define RESETENCODERS       0x20
#define LEFTENCODER         0x02          // Byte to read motor encodeur 1
#define RIGHTENCODER        0x06          // Byte to read motor encodeur 2
#define ACCELERATION        14
#define LED_BUILTIN         13



class BaseRoulante
{
public :
    //Constructor :
    BaseRoulante(float empattement, float rayonRoues); //empattement, rayon

    void avancerTourner(int v, int theta);          // vitesseMoyenne, vitesseRotation
    bool tournerPrecis(float theta, float precision);       // angle, precision
    bool suivrePoint(float xCible, float yCible, float precision);   //x, y, precision
    void allerRetour(float x,float y); //x, y
    void actualiserPosition();
    void setDebugOn();
    void setDebugOff();
    void setPosition(double , double); //posX, posY
    void setPosition(double, double, double); //posX, posY et angle
    double getPosX();
    double getPosY();
    double getAngle();
    void setAcceleration(byte acc);
    void setAngleCorrecteur(float kp, float ki, float kd);       //P, I, D
    void setDistanceCorrecteur(float kp, float ki, float kd);    //P, I, D


    int getBattery();
    void setMode(byte mode);
    void resetEncoders();
    void setAutoRegulation();
    void setNoAutoRegulation();
    void setAutoStopMotors();
    void setNoAutoStopMotors();
    double getEncoder(byte encoder);
    float moduloPI(float angle);


private :

    float m_empattement = 25.5 / 2;        //distance entre le centre de rotation du robot et une roue en cm !
    float m_rayon = 9.9 / 2;         //rayon des roues en cm
    double m_angle = 0; // = 0                        // Sotck l'angle du robot au cours du temps, il va quand mÃªme rester Ã  dÃ©finir un cotÃ© + et un -
    double m_posX = 0; //
    double m_posY = 0; // =
    double m_encodeurPrecedentGauche = 0;
    double m_encodeurPrecedentDroit = 0;

    //Variables pour les correcteurs d'asservissement
    double m_consigneAngle; // consigne pour le correcteur PID en angle
    double m_vitesseRotation; // sortie du PID pour régler l'angle absolu du robot
    float m_Kp = 30; //70
    float m_Ki = 1;  //1
    float m_Kd = 0;

    double m_consigneDistance = 0; //consigne pour le PID en distance
    double m_distance;   // entrée du PID
    double m_vitesseMoyenne; // sotie du PID
    float m_dKp = 2; //2.5
    float m_dKi = 1;
    float m_dKd = 0;

    //variables pour le maniement séquentiel du robot
    int m_compteur = 0;
    int m_etape = 1; //permet de savoir où en est le robot dans la rÃ©alisation des taches
    int m_nbconf = 50; // le nombre de temps que le robot doit Ãªtre stable une fois la tache rÃ©ussi avant de passer Ã  la suivante
    bool debug = true;

    PID correctionAngle = PID(&m_angle, &m_vitesseRotation, &m_consigneAngle, m_Kp, m_Ki, m_Kd);
    PID correctionDistance = PID(&m_distance, &m_vitesseMoyenne, &m_consigneDistance, m_dKp, m_dKi, m_dKd);


};
#endif // BASEROULANTE_H






















