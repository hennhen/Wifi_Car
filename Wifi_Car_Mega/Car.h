/*
  Car.h - Library for my Wifi RC Car

  - Written to simplify the process of controlling the car
  - Controls Motor and Servo objects
  
  Created by Henry Wu
  April 4, 2019
*/

#ifndef Car_h
#define Car_h

#include "Motor.h"
#include <Servo.h>
#include <Servo.h>

class Car {

  public:
    Motor motor;
    Servo servo;

    Car(Motor& m, Servo& s);
    void init();								    // Initializes the car. Init motor and servo (attached to pin 9)
    void drive(int spd);						// Drive with speed (PWM, negative is backwards)
    void turn(int8_t angle);				// Right is positive
    void pause();								    // Stop, reset servo

  private:
    int _pin;
};

#endif
