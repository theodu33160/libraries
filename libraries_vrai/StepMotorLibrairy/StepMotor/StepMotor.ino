class StepMotor
{
    int m_voieA;
    int m_voieB;
    int m_voieC;
    int m_voieD;
    int m_voieP;
    unsigned long previousMillis;

  public:
    StepMotor(int voieA, int voieB, int voieC, int voieD)
    {

      m_voieA = voieA;
      m_voieP = voieA;
      m_voieB = voieB;
      m_voieC = voieC;
      m_voieD = voieD;

      pinMode(m_voieA, OUTPUT);
      pinMode(m_voieB, OUTPUT);
      pinMode(m_voieC, OUTPUT);
      pinMode(m_voieD, OUTPUT);

      previousMillis = 0;
    }

    void one_step(int voie)
    {
      for (int i = m_voieA; i <= m_voieD; i++)
      {
        digitalWrite(i, LOW);
      }
      digitalWrite(voie, HIGH);
      if (voie == m_voieD)
      {
        digitalWrite(m_voieA, HIGH);
      }
      else
      {
        digitalWrite(voie + 1, HIGH);
      }
    }

    void Update()
    {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= 2)
      {
        one_step(m_voieP);
        if (m_voieP == m_voieD)
        {
          m_voieP = m_voieA;
        }
        else
        {
          m_voieP++;
        }
        previousMillis = currentMillis;
      }
    }

    long get_time()
    {
      return previousMillis;
    }
};
