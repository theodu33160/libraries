#include <TestClass.h>

TestClass::TestClass(bool etat)
{
    //if(!Serial.available()) Serial.begin(1000000);
    //Serial.println("SETUP ... ");

    m_etat = etat;
}

void TestClass::main()
{
#if defined(BLINK1)
    if(!Serial.available()) Serial.begin(1000000);
    Serial.println("BLINK");
    if(millis()%100<50)
    {
        digitalWrite(LED_BUILTIN,LOW);
    }
    else
    {
        digitalWrite(LED_BUILTIN,HIGH);
    }
#else
    if(!Serial.available()) Serial.begin(1000000);
    Serial.println("NO BLINK");
    if(millis()%1000<500)
    {
        digitalWrite(LED_BUILTIN,LOW);
    }
    else
    {
        digitalWrite(LED_BUILTIN,HIGH);
    }
#endif
    if(!Serial.available()) Serial.begin(1000000);
    Serial.println("CACA");
}
