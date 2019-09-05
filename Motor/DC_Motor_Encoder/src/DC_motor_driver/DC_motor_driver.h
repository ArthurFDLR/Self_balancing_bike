/**
 * Adapted from CytronTechnologies library to control their motor drivers
 * Add PWM limitation to control a 6v motor with 12v battery
 *
 * author : Arthur FINDELAIR, github.com/ArthurFDLR
 * date : August 2019
 */

#ifndef DC_motor_driver_h
#define DC_motor_driver_h

#include <Arduino.h>

class DC_motor
{
  public:
	  DC_motor(uint8_t pin1, uint8_t pin2, uint8_t _PWM_limit);
	  void setSpeed(int16_t speed);

  private:
	  uint8_t _PWM_limit;
	  uint8_t _pin1;
	  uint8_t _pin2;
};

#endif