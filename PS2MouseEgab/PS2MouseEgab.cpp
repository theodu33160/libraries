#include "PS2MouseEgab.h"

PS2MouseEgab::PS2MouseEgab(int clk, int data, int n){
  _ps2clk=clk;
  _ps2data=data;
  gohi(_ps2clk);
  gohi(_ps2data);
  m_n=n;
  m_x=0;
  m_y=0;

}

void PS2MouseEgab::write(uint8_t data){ // Attention prend au moins 660 µs
  uint8_t parity=1;
  int temps_absolu = micros();
  gohi(_ps2data);
  gohi(_ps2clk);
  delayMicroseconds(300);
  golo(_ps2clk);
  delayMicroseconds(300);
  golo(_ps2data);
  delayMicroseconds(10);
  gohi(_ps2clk);
  
  while(digitalRead(_ps2clk)==HIGH);
  
  for(int i=0; i<8; i++){
    if(data&0x01) gohi(_ps2data);
    else golo(_ps2data); 
    while(digitalRead(_ps2clk)==LOW);
    while(digitalRead(_ps2clk)==HIGH);
    parity^=(data&0x01);
    data=data>>1;
  }
  
  if(parity) gohi(_ps2data);
  else golo(_ps2data);

  while(digitalRead(_ps2clk)==LOW);
  while(digitalRead(_ps2clk)==HIGH);
  
  gohi(_ps2data);
  delayMicroseconds(50);
  
  while(digitalRead(_ps2clk)==HIGH);
  while((digitalRead(_ps2clk)==LOW)||(digitalRead(_ps2data)==LOW));
  
  golo(_ps2clk);
}

uint8_t PS2MouseEgab::read(void){   // Attention prend au moins 55 µs
  uint8_t data=0, bit=1;
 
  gohi(_ps2clk);
  gohi(_ps2data);
  delayMicroseconds(50);
  while(digitalRead(_ps2clk)==HIGH);
  
  delayMicroseconds(5);
  while(digitalRead(_ps2clk)==LOW);
  
  for(int i=0; i<8; i++){
    while(digitalRead(_ps2clk)==HIGH);
    bit=digitalRead(_ps2data);
    while(digitalRead(_ps2clk)==LOW);
    bitWrite(data,i,bit);
  }
  
  while(digitalRead(_ps2clk)==HIGH);
  while(digitalRead(_ps2clk)==LOW);
  while(digitalRead(_ps2clk)==HIGH);
  while(digitalRead(_ps2clk)==LOW);
  
  golo(_ps2clk);
  
  return data;
}

void PS2MouseEgab::begin(void){
  write(0xFF);
  for(int i=0; i<3; i++) read();
  write(0xF0);
  read();
  delayMicroseconds(100);
}

void PS2MouseEgab::actualiserPosition(){
  write(0xEB); // Attention prend 660 µs
  read();      // Attention prend 55 µs
  m_stat=read(); // Attention prend 55 µs
  uint8_t dataX=read(); // Attention prend 55 µs
  uint8_t dataY=read(); // Attention prend 55 µs Donc au total l'actualisation de la position d'une souris prend 880µs ~ 1ms (ça passe mais voir si on peut optimiser)
  bool negx=bitRead(m_stat,4);
  bool negy=bitRead(m_stat,5);

  int DX = (int)dataX;
  int DY = (int)dataY;
  if(negx) DX|=0xFF00;
  if(negy) DY|=0xFF00;
  m_dx=((float)DX/m_n);
  m_dy=((float)DY/m_n);
  m_x+=m_dx;
  m_y+=m_dy;

}

void PS2MouseEgab::golo(int pin){
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void PS2MouseEgab::gohi(int pin){
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}


float PS2MouseEgab::getPositionX(){
    return m_x;
}

float PS2MouseEgab::getPositionY(){
    return m_y;
}

float PS2MouseEgab::getDX(){
    return m_dx;
}

float PS2MouseEgab::getDY(){
    return m_dy;
}

void  PS2MouseEgab::setPositionX(float x){
    m_x=x;
}
void  PS2MouseEgab::setPositionY(float y){
    m_x=y;
}
