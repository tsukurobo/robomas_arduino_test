#include "robomas.h"

void setup(){
  Serial.begin(9600);
  C610::begin();
}

void loop(){
  C610::put(1, 1000);
  C610::put(2, 1000);

  for(int i= 0; i<8; i++){
    //Serial.print(C610::received_angle[i]);
    Serial.print(C610::getAngle(i+1));
    Serial.print(" ");
  }

  Serial.println();
  C610::send();
  C610::receive();
  delay(100);
}