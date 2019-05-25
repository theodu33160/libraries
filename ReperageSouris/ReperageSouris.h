#include <PS2MouseEgab.h>

#ifndef REPERAGESOURIS_H
#define REPERAGESOURIS_H

#include <Arduino.h>


class ReperageSouris {
  public:

    ReperageSouris(PS2MouseEgab* mouse1, PS2MouseEgab *mouse2, float alpha, float distanceSouris); // Alpha Angle entre le repère de la première souris et celui de la deuxième.

    void actualiserPosition(void);
    float getPositionX(void);
    float getPositionY(void);
    float getAngle(void);
    void setPositionX(float x);
    void setPositionY(float y);
    void setAngle(float theta);

private:

    float m_x=0;
    float m_y=0;
    float m_theta=0;
    float m_alpha;
    float m_distanceSouris;

    PS2MouseEgab* m_mouse1;
    PS2MouseEgab* m_mouse2;

};

#endif // REPERAGESOURIS_H
