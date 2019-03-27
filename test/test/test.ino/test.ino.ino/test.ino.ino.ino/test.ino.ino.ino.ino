void setup() {
  // put your setup code here, to run once:
Serial.begin(1000000);
}

void loop() {
  unsigned long t1;
  unsigned long t0 = micros();
  for(int i =0;i<1000;i++)
  { 
    if(!Serial.available()) {}
  }
  t1 =micros();
  Serial.println(t1-t0);
  // put your main code here, to run repeatedly:

}
