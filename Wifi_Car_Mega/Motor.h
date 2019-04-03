/*
	Motor.h - Library to control Motor with Pololu Shielf

	Henry Wu
*/ 
#ifndef Motor_h
#define Motor_h

#include "Motor.h"

class Motor {
    

  public:
    Motor(uint8_t  slpPin, uint8_t pwmPin, uint8_t dirPin);
	void init();
	void forward();
	void forward(uint8_t spd);
	void backward();
	void backward(uint8_t spd);
	void setSbeed(uint8_t spd);
	
  private: 
	const uint8_t slpPin, pwmPin, dirPin;
};

#endif
