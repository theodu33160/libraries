#ifndef TESTCLASS_H
#define TESTCLASS_H

#if defined (BLINK)
    #define BLINK1
#endif


#include "Arduino.h"


class TestClass
{
public :
    TestClass(bool etat);
    void main();

private :
    bool m_etat;

};
#endif // TESTCLASS_H
