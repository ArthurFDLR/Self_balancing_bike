#include "src/DC_motor_driver/DC_motor_driver.h"
#include "src/Encoder/Encoder.h"

//#include <Ewma.h> // Filtering library

/*----------------------------------*/
/*   VARIABLES and INSTANTIATIONS   */
/*----------------------------------*/

//// GLOBAL  ////

unsigned long timePrev = 0;
const int workingFrequency = 15; //Hz

//// DC Motor control ////

const int pinPotentiometer = 0; // A0
const int pinEscPWM = 11;
const int pinEscDir = 12;
const int limitMotorPwm = 128; //Stay under 6V for 12V alim.

DC_motor motor( pinEscPWM, pinEscDir, limitMotorPwm);  // PWM = Pin 11, DIR = Pin 12.
int motorPwm = 0;

//// DC Motor speed ////

/*
const float motorFilter = 0.5;       // [0,1] exponential filter parameter
const int encoderCountRev = 212;
const int pinEncoderAInterrupt = 3; //PD3
const int pinEncoderB = 4;  //PD4

const char registerMaskPIND = 0b00011000; //Show pin 
volatile long motorTickCount = 0;
unsigned long motorTimePrev = 0; //Some of those variables can be ignored/simplify
unsigned long motorTimeNow = 0;
unsigned long motorTimeDiff = 0;
unsigned long motorSpeedRpm = 0;
unsigned long motorSpeedRpmPrev = 0;
unsigned long motorSpeedRpmRaw = 0;
int motorDirection = 0; // {-1;0;1} with 0 => stopped


void ISR_updateEncoder(){ //Direct access to register to optimize time consumption
  if (((registerMaskPIND & PIND) == 0b00010000) or ((registerMaskPIND & PIND) == 0b00001000)){ 
    motorTickCount++;
  } else if (((registerMaskPIND & PIND) == 0b00000000) or ((registerMaskPIND & PIND) == 0b00011000)){
    motorTickCount--;
  }
}


void Update_MotorData(){

  //Get motor rotation speed from tick count
  motorTimeNow = millis();
  motorTimeDiff = motorTimeNow - motorTimePrev;
  motorTimePrev = motorTimeNow;
  motorSpeedRpmPrev = motorSpeedRpm;
  motorSpeedRpmRaw = ((motorTickCount > 0) ? motorTickCount : -motorTickCount) * 60000 / (encoderCountRev * motorTimeDiff); // 60000 => ms to s
  motorTickCount = 0;

  //Set motor rotation direction
  if (motorSpeedRpm == 0) {
    motorDirection = 0;
  } else if (motorTickCount > 0) {
    motorDirection = 1;
  } else {
    motorDirection = -1;
  }
  
  //Filtering
  motorSpeedRpm = motorSpeedRpmRaw * motorFilter + motorSpeedRpmPrev * (1.0 - motorFilter);
}
*/

EncoderQuad encoderInertiaWheel(3,4,0.5,212);

/*-----------------------*/
/*   Arduino Processes   */
/*-----------------------*/

void setup() {
  Serial.begin(9600);
/*
  pinMode(pinEncoderAInterrupt, INPUT_PULLUP);
  pinMode(pinEncoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncoderAInterrupt), ISR_updateEncoder, CHANGE);
  
  motorTimePrev = millis();

  motorTickCount = 0;
*/
  timePrev = millis();
}

void loop() {
  if (millis() - timePrev > 1000/workingFrequency) {
    
    timePrev = millis();
    motorPwm = map(analogRead(A0), 0, 1023, 0, limitMotorPwm);
    motor.setSpeed(motorPwm);

    encoderInertiaWheel.Update_MotorData();

    Serial.print(motorPwm);
    Serial.print("\t");
    Serial.println(encoderInertiaWheel.motorSpeedRpm);
    
  }
}
