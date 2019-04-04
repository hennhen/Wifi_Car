
/*
    Motor.cpp Library

    Author: Henry Wu
*/

#include "Arduino.h"
#include "Motor.h"

Motor::Motor(uint8_t  slpPin, uint8_t pwmPin, uint8_t dirPin) :  slpPin(slpPin), pwmPin(pwmPin), dirPin(dirPin) {
  pinMode(Motor::slpPin, OUTPUT);
  pinMode(Motor::pwmPin, OUTPUT);
  pinMode(Motor::dirPin, OUTPUT);
}

void Motor::init() {
  digitalWrite(slpPin, HIGH);                         // Wake up the controller
  delay(20);
  digitalWrite(dirPin, HIGH);                         // Set dir. to forward
  analogWrite(pwmPin, 0);                             // 0 speed
}

void Motor::forward() {                                      // Set dir. to forward. Maintain current speed.
  digitalWrite(dirPin, HIGH);
}

void Motor::forward(uint8_t spd) {                           // Go forward with input speed
  digitalWrite(dirPin, HIGH);
  analogWrite(pwmPin, spd);
}

void Motor::backward() {
  digitalWrite(dirPin, LOW);
}

void Motor::backward(uint8_t spd) {
  digitalWrite(dirPin, LOW);
  analogWrite(pwmPin, spd);
}

void Motor::setSbeed(uint8_t spd) {
  analogWrite(pwmPin, spd);
}
