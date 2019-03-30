#include "ReperageSouris.h"


ReperageSouris::ReperageSouris(PS2MouseEgab* mouse1, PS2MouseEgab* mouse2, float alpha, float distanceSouris){

  m_mouse1 = mouse1;
  m_mouse2 = mouse2;
  m_x=0;
  m_y=0;
  m_theta=0;
  m_alpha=alpha;
  m_distanceSouris=distanceSouris;

}

void  ReperageSouris::actualiserPosition(){
     m_mouse1->actualiserPosition();
     m_mouse2->actualiserPosition();

     float dx1=m_mouse1->getDX();//récupération de l'abscisse de la souris 1 dans son repère (celui de référence par convention).
     float dy1=m_mouse1->getDY();//récupération de l'ordonnée de la souris 1 dans son repère (celui de référence par convention).

     float dX2=m_mouse2->getDX();//récupération de l'abscisse de la souris 2 dans son repère.
     float dY2=m_mouse2->getDY();//récupération de l'abscisse de la souris 2 dans son repère.

     float dx2=dX2*cos(m_alpha)-dY2*sin(m_alpha); //calcul des coordonées de la souris 2 dans le repère de la souris 1.
     float dy2=dX2*sin(m_alpha)+dY2*cos(m_alpha);

     m_theta+=atan((dy1-dy2)/m_distanceSouris); //calcul de la position angulaire du robot en degré.

     float dX=(dx1+dx2)/2; // calcul de la position du centre du robot /!\ On considère sue le centre du robot est le milieu du segment reliant les deux souris
     float dY=(dy1+dy2)/2;

     m_x += dX*cos(m_theta)+dY*sin(m_theta);
     m_y += -dX*sin(m_theta)+dY*cos(m_theta);

}

float ReperageSouris::getPositionX(){
    return m_x;
}

float ReperageSouris::getPositionY(){
    return m_y;
}

float ReperageSouris::getAngle(){
    return (180/PI)*m_theta;

}

void  ReperageSouris::setPositionX(float x){
    m_x=x;
}

void  ReperageSouris::setPositionY(float y){
    m_x=y;
}


void  ReperageSouris::setAngle(float theta){
    m_theta=theta;
}
