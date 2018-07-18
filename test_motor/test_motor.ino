/*
sketch/Import Library
-- Add the folder AFMOTOR 
---git/dependecies/
*/

#include <AFMotor.h> // https://learn.adafruit.com/adafruit-motor-shield/library-install

AF_DCMotor motorEsq(1, MOTOR12_64KHZ);
AF_DCMotor motorDir(2, MOTOR12_64KHZ);

void setup() {
  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
  Serial.begin(9600);
}

void loop() {
 motorDir.setSpeed(50);
 motorEsq.setSpeed(50); 
 motorDir.run(FORWARD);
 motorEsq.run(FORWARD);
  
}
