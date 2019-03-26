#include "ReperageSouris.h"
#include "PS2MouseEgab.h"
//---------PARAMETRES ROBOT------------------

const byte largeur = 30; //Largeur du robot en cm (incluant les roues)
const byte longueur = 30; //Longeur du robot en cm (incluant les roues)

//-------------BRANCHEMENTS----------------
byte pinHorloge1=3;
byte pinData1=2;
byte pinHorloge2=5;
byte pinData2=4;

//-------VARIABLES POSITION ROBOT------

float x=largeur/2;
float y=longueur/2;
float theta=0;

//----PARAMETRES SOURIS------------------
const float n1=3360/200;
const float n2=3360/200;
float alpha = 0; // Angle entre le repère de la première souris et celui de la deuxième.

float distanceSouris=60;

PS2MouseEgab mouse1(pinHorloge1,pinData1,n1);// définition des deux objets souris
PS2MouseEgab mouse2(pinHorloge2,pinData2,n2);

ReperageSouris souris = ReperageSouris(&mouse1, &mouse2, alpha, distanceSouris);

void setup(){
  Serial.begin(9600); // initialisation des deux souris 
  while(!Serial);
  Serial.print("Initialisation souris 1...");
  mouse1.begin();
  Serial.println("souris 1 OK");
  Serial.print("Initialisation souris 2...");
  mouse2.begin();
  Serial.println("souris 2 OK");
  mouse2.setPositionX(largeur);
}

void loop(){
  souris.actualiserPosition();
  x=souris.getPositionX();
  y=souris.getPositionY();
  theta=souris.getAngle();
  Serial.print("\tx=");
  Serial.print(x);
  Serial.print("\ty=");
  Serial.print(y);
  Serial.print("\ttheta=");
  Serial.println(theta);
  
}
