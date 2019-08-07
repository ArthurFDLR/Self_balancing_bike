#include "src/DC_motor_driver/DC_motor_driver.h"
#include <Encoder.h>

/*----------------------------------*/
/*   VARIABLES and INSTANTIATIONS   */
/*----------------------------------*/

//// DC Motor control ////

const int pinPotentiometer = 0; // A0
const int pinEscPWM = 11;
const int pinEscDir = 12;
const int limitMotorPwm = 128; //Stay under 6V for 12V alim.
DC_motor motor( pinEscPWM, pinEscDir, limitMotorPwm);  // PWM = Pin 11, DIR = Pin 12.
int motorPwm = 0;

//// DC Motor speed ////

const int motorIntervalMin = 100; //ms
const int encoderCountRev = 212; // verified with tachymeter
const int pinEncoderAInterrupt = 3;
const int pinEncoderB = 4;
Encoder myEnc(pinEncoderAInterrupt, pinEncoderB); // Only one interupt pin. Need the other one for the IMU
long motorTickCount = 0;
unsigned long motorTimePrev = 0;
unsigned long motorTimeNow = 0;
unsigned long motorTimeDiff = 0;
unsigned long motorSpeedRpm = 0;
unsigned long motorSpeedRpmPrev = 0;
int motorDirection = 0; // {-1;0;1} with 0 => stopped

void Update_MotorData(){
  
  motorTimeNow = millis();
  if (motorTimeNow - motorTimePrev > motorIntervalMin){
    motorTimeDiff = motorTimeNow - motorTimePrev;
    motorTimePrev = motorTimeNow;
    motorSpeedRpmPrev = motorSpeedRpm;
    motorTickCount = myEnc.read();
    motorSpeedRpm = (float)(((motorTickCount>0) ? motorTickCount : -motorTickCount) * 60000 / (encoderCountRev * motorTimeDiff));
    if (motorSpeedRpm==0){
      motorDirection = 0;
    } else if (motorTickCount>0){
      motorDirection = 1;
    } else {
      motorDirection = -1;
    }
    myEnc.write(0);
  }
}
/*-----------------------*/
/*   Arduino Processes   */
/*-----------------------*/

void setup() {
  Serial.begin(9600);

}

void loop() {
  motorPwm = map(analogRead(pinPotentiometer), 0, 1023, 0, limitMotorPwm);
  motor.setSpeed(motorPwm);

  Update_MotorData();
  
  Serial.print(motorPwm);
  Serial.print("\t");
  Serial.print(motorDirection);
  Serial.print("\t");
  Serial.println(motorSpeedRpm);
}
