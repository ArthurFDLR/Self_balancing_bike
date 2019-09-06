#include "DC_motor_driver.h"

DC_motor::DC_motor(uint8_t pin1, uint8_t pin2, uint8_t PWM_limit)
{
  _pin1 = pin1;
  _pin2 = pin2;
  _PWM_limit = PWM_limit;

  pinMode(_pin1, OUTPUT);
  pinMode(_pin2, OUTPUT);
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, LOW);
}

void DC_motor::setSpeed(int16_t speed)
{
  // Make sure the speed is within the limit.
  if (speed > _PWM_limit) {
    speed = 255;
  } else if (speed < -_PWM_limit) {
    speed = -255;
  }
  
  // Set the speed and direction.
  if (speed >= 0) {
		analogWrite(_pin1, speed);
		digitalWrite(_pin2, LOW);
  } else {
		analogWrite(_pin1, -speed);
		digitalWrite(_pin2, HIGH);
  }

}
