/**
   - Run on arduino nano
   - Controle and measure DC motor's speed
   - Read IMU data
   
   author : Arthur FINDELAIR, github.com/ArthurFDLR
   date : September 2019
*/

#include "src/DC_motor_driver/DC_motor_driver.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/*----------------------------------*/
/*   VARIABLES and INSTANTIATIONS   */
/*----------------------------------*/

//// GLOBAL  ////

uint16_t timePrev = 0;
uint16_t computationTime = 0;
const int8_t workingFrequency = 65; //Hz

//// Encoder  ////

uint16_t motorSpeedRpm = 0;
int8_t motorDirection = 0; // {-1;0;1} with 0 => stopped

//// IMU  ////

float ypr[3]; // [yaw, pitch, roll]

//// DC Motor control ////

const int8_t pinPotentiometer = 0; // A0
const int8_t pinEscPWM = 11;
const int8_t pinEscDir = 12;
const int8_t limitMotorPwm = 128; //Stay under 6V for 12V alim.

DC_motor motor( pinEscPWM, pinEscDir, limitMotorPwm);  // PWM = Pin 11, DIR = Pin 12.
int8_t motorPwm = 0;


/*-----------------------*/
/*   Arduino Processes   */
/*-----------------------*/

void setup() {
  Wire.begin(); // join I2C bus (I2Cdev library doesn't do this automatically)
  //Wire.setClock(400000); // 400kHz I2C clock. By default : 100kHz
  Serial.begin(9600);

  Encoder_Setup();
  IMU_setup();

  timePrev = millis();
}

void loop() {
  if (millis() - timePrev > 1000 / workingFrequency) {

    timePrev = millis();
    motorPwm = map(analogRead(A0), 0, 1023, 0, limitMotorPwm);
    motor.setSpeed(motorPwm);

    Update_MotorData();
    Update_YPR();

    computationTime = millis() - timePrev;
    
    Serial_viewer();    
  }
  else {
    Serial.println("_"); //Check working frequency calibration
  }
}
