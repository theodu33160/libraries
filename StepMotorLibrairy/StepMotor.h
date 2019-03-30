#ifndef StepMotor
#define StepMotor
#include "Arduino.h"

class StepMotor
{
  public:
    StepMotor(int voieA, int voieB, int voieC, int voieD);
    void one_step(int voie)
    void Update();
  private:
        int m_voieA;
    int m_voieB;
    int m_voieC;
    int m_voieD;
    int m_voieP;
    unsigned long previousMillis;
};

#endif