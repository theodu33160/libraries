#ifndef PS2MOUSEEGAB_H
#define PS2MOUSEEGAB_H

#include <Arduino.h>

class PS2MouseEgab {
  public:
    PS2MouseEgab(int data, int clk, int n);
    void write(uint8_t data);
    uint8_t read(void);

    void begin(void);
    void actualiserPosition(void);
    float getPositionX(void);
    float getPositionY(void);

    float getDX(void);
    float getDY(void);

    void setPositionX(float x);
    void setPositionY(float y);

private:
    uint8_t m_stat;
    float m_x=0;
    float m_y=0;

    float m_n;
    float m_dx;
    float m_dy;

    int _ps2clk;
    int _ps2data;
    void golo(int pin);
    void gohi(int pin);
};

#endif // PS2MOUSEEGAB_H
