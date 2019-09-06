/**
 * Run on arduino nano to controle and measure DC motor's speed
 *
 * author : Arthur FINDELAIR, github.com/ArthurFDLR
 * date : September 2019
 */

#include "src/DC_motor_driver/DC_motor_driver.h"
#include "src/Encoder/Encoder.h"

//#include <Ewma.h> // Filtering library

/*----------------------------------*/
/*   VARIABLES and INSTANTIATIONS   */
/*----------------------------------*/

//// GLOBAL  ////

unsigned long timePrev = 0;
const int workingFrequency = 15; //Hz

unsigned long motorSpeedRpm = 0;
int motorDirection = 0; // {-1;0;1} with 0 => stopped

//// DC Motor control ////

const int pinPotentiometer = 0; // A0
const int pinEscPWM = 11;
const int pinEscDir = 12;
const int limitMotorPwm = 128; //Stay under 6V for 12V alim.

DC_motor motor( pinEscPWM, pinEscDir, limitMotorPwm);  // PWM = Pin 11, DIR = Pin 12.
int motorPwm = 0;

//// Encoder ////

EncoderQuad encoderWheel(3,4,0.5,212);

void ISR_TickEncoder(){ //ISR interface function
  encoderWheel.ISR_updateEncoder();
}

/*-----------------------*/
/*   Arduino Processes   */
/*-----------------------*/

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(3), ISR_TickEncoder, CHANGE);
  timePrev = millis();
}

void loop() {
  if (millis() - timePrev > 1000/workingFrequency) {
    
    timePrev = millis();
    motorPwm = map(analogRead(A0), 0, 1023, 0, limitMotorPwm);
    motor.setSpeed(motorPwm);

    encoderWheel.Update_MotorData();

    Serial.print(motorPwm);
    Serial.print("\t");
    Serial.println(encoderWheel.motorSpeedRpm);
    
  }
}
