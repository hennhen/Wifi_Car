/*
  Car.h - Library for RC Car
  Created by Henry Wu
*/

#include "Arduino.h"
#include "Car.h"
#include "Motor.h"
#include "Servo.h"

	Car::Car(Motor& m, Servo& s) : motor(m), servo(s) { }
	
	void Car::init() {
      motor.init();                                   // Wake up, forward, 0 speed
      servo.attach(9);                                // Servo Pin
      servo.write(90);                                // Center the servo
    }

    void Car::drive(int spd){
      if(spd >= 0){
        motor.forward(spd);
      }
      else if(spd < 0){
        motor.backward(-spd);
      }
    }

    void Car::turn(int8_t angle){                          // Right = positive
       servo.write(90 + angle);
    }

    void Car::pause(){
      motor.setSbeed(0);
      servo.write(90);
    }
