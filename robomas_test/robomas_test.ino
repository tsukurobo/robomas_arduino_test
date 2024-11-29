#include "robomas.h"

void setup(){
  Serial.begin(9600);
  C610::begin();
}

void loop(){
  C610::put(3, 0);
  C610::send();
  C610::receive();
  for(int i= 0; i<8; i++){
    //Serial.print(C610::received_angle[i]);
    Serial.print(C610::getAngle(i+1));
    Serial.print(" ");
  }
  Serial.println();
  delay(100);
}