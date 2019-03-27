#include "ReperageSouris.h"


ReperageSouris::ReperageSouris(PS2MouseEgab* mouse1, PS2MouseEgab* mouse2, float alpha){
  m_mouse1 = mouse1;
  m_mouse2 = mouse2;
  m_x=0;
  m_y=0;
  m_theta=0;
  m_alpha=alpha;
}

void  ReperageSouris::actualiserPosition(){
     m_mouse1->actualiserPosition();
     m_mouse2->actualiserPosition();

     float x1=m_mouse1->getPositionX();//récupération de l'abscisse de la souris 1 dans son repère (celui de référence par convention).
     float y1=m_mouse1->getPositionY();//récupération de l'ordonnée de la souris 1 dans son repère (celui de référence par convention).

     float X2=m_mouse2->getPositionX();//récupération de l'abscisse de la souris 2 dans son repère.
     float Y2=m_mouse2->getPositionY();//récupération de l'abscisse de la souris 2 dans son repère.


     float x2=X2*cos(m_alpha)-Y2*sin(m_alpha); //calcul des coordonées de la souris 2 dans le repère de la souris 1
     float y2=X2*sin(m_alpha)+Y2*cos(m_alpha);

     m_x+=(x1+x2)/2; // calcul de la position du centre du robot /!\ On considère sue le centre du robot est le milieu du segment reliant les deux souris
     m_y+=(y1+y2)/2;
     if(x2==x1){
         m_theta=(2*(y2>y1)-1)*(PI/2);
     }
     else {
         m_theta=atan((y2-y1)/(x2-x1)); //calcul de la position angulaire du robot
     }
}

float ReperageSouris::getPositionX(){
    return m_x;
}

float ReperageSouris::getPositionY(){
    return m_y;
}

float ReperageSouris::getAngle(){
    return m_y;
}

void  ReperageSouris::setPositionX(float x){
    m_x=x;
}

void  ReperageSouris::setPositionY(float y){
    m_x=y;
}

void  ReperageSouris::setAngle(float alpha){
    m_alpha=alpha;
}
