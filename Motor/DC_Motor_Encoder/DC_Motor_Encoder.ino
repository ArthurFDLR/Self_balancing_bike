#include "src/DC_motor_driver/DC_motor_driver.h"
#include <Encoder.h>

/*----------------------------------*/
/*   VARIABLES and INSTANTIATIONS   */
/*----------------------------------*/

//// GLOBAL  ////

unsigned long timePrev = 0;
const int workingFrequency = 20; //Hz

//// DC Motor control ////

const int pinPotentiometer = 0; // A0
const int pinEscPWM = 11;
const int pinEscDir = 12;
const int limitMotorPwm = 128; //Stay under 6V for 12V alim.

DC_motor motor( pinEscPWM, pinEscDir, limitMotorPwm);  // PWM = Pin 11, DIR = Pin 12.
int motorPwm = 0;

//// DC Motor speed ////

//const int motorIntervalMin = 25; //ms
const float motorFilter = 0.8;       // [0,1] exponential filter parameter
const int encoderCountRev = 212; // verified with tachymeter
const int pinEncoderAInterrupt = 3;
const int pinEncoderB = 4;

Encoder myEnc(pinEncoderAInterrupt, pinEncoderB); // Only one interupt pin. Need the other one for the IMU
long motorTickCount = 0;
unsigned long motorTimePrev = 0; //Some of those variables can be ignored simplify
unsigned long motorTimeNow = 0;
unsigned long motorTimeDiff = 0;
unsigned long motorSpeedRpm = 0;
unsigned long motorSpeedRpmPrev = 0;
unsigned long motorSpeedRpmRaw = 0;
int motorDirection = 0; // {-1;0;1} with 0 => stopped

void Update_MotorData() {
  //if (millis() - motorTimePrev > motorIntervalMin) {

  //Get motor rotation speed from tick count
  motorTimeNow = millis();
  motorTimeDiff = motorTimeNow - motorTimePrev;
  motorTimePrev = motorTimeNow;
  motorSpeedRpmPrev = motorSpeedRpm;
  motorTickCount = myEnc.read();
  motorSpeedRpmRaw = (((motorTickCount > 0) ? motorTickCount : -motorTickCount) * 60000 / (encoderCountRev * motorTimeDiff)); // 60000 => ms to s
  myEnc.write(0);

  //Set motor rotation direction
  if (motorSpeedRpm == 0) {
    motorDirection = 0;
  } else if (motorTickCount > 0) {
    motorDirection = 1;
  } else {
    motorDirection = -1;
  }

  //Exponential filtering
  motorTickCount = 0;
  motorSpeedRpm = motorSpeedRpmRaw * motorFilter + motorSpeedRpmPrev * (1.0 - motorFilter);
}
/*-----------------------*/
/*   Arduino Processes   */
/*-----------------------*/

void setup() {
  Serial.begin(9600);
  pinMode(pinEncoderAInterrupt, INPUT_PULLUP);
  pinMode(pinEncoderB, INPUT_PULLUP);
  motorTimePrev = millis();
  timePrev = millis();
}

void loop() {
  if (millis() - timePrev > 1000/workingFrequency) {
    
    timePrev = millis();
    motorPwm = map(analogRead(pinPotentiometer), 0, 1023, 0, limitMotorPwm);
    motor.setSpeed(motorPwm);

    Update_MotorData();

    Serial.print(motorSpeedRpmRaw);
    Serial.print("\t");
    Serial.print(motorSpeedRpmPrev);
    Serial.print("\t");
    Serial.println(motorSpeedRpm);
    
  }
}
