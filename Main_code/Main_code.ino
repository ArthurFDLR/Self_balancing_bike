/**   SELF STABILIZED BIKE

   note : - Run on arduino nano
          - Controle and measure DC motor's speed
          - Read IMU data
          - Compute PIDs data

   author : Arthur FINDELAIR, github.com/ArthurFDLR
   date : September 2019
*/

// =============================
// ===       Libraries       ===
// =============================

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

// ================================================
// ===       VARIABLES and INSTANTIATIONS       ===
// ================================================

/*--------------*/
/*   GLOBAL     */
/*--------------*/

char machineState = '0';
char serialViewerMode = '0';
uint16_t timePrev = 0;
uint16_t computationTime = 0;
const int8_t workingFrequency = 65; //Hz

/*----------------------*/
/*   PID controller     */
/*----------------------*/

// Tunings
uint32_t KP = 50;
uint32_t KI = 0;
uint32_t KD = 0;
uint32_t KS = 0;

uint32_t motorFriction = 0;

uint32_t setPoint_KP = 0; //100
uint32_t setPoint_KD = 0;  //45

// Exponential filter parameters
const float leanAngle_Filter = 0.9;
const float leanAngle_HeavyFilter = 0.4;
const float setPoint_filter = 0.8;
const float leanAngle_Derivative_Filter = 0.6;

// Variables
float leanAngle; //degrees
float leanAngle_raw;
float leanAngle_prev;
float leanAngle_smoothed;
float leanAngle_smoothed_prev;
float leanAngleError = 0;

float leanAngleSetPoint = 2.7;
float leanAngleSetPoint_prev = 0;
float leanAngleSetPoint_raw = 0;

float leanAngle_derivative;
float leanAngle_derivative_prev;
float leanAngle_derivative_smoothed;
float leanAngle_derivative_smoothed_prev;

float leanAngle_Integer;

double P_control, I_control, D_control, S_control;
double PID_output = 0;

float PID_friction;

/*---------------*/
/*   ENCODER     */
/*---------------*/

const float motorFilter = 0.5;       // [0,1] exponential filter parameter
const float encoderCountRev = 212.0;
const int8_t pinEncoderAInterrupt = 3; //PD3
const int8_t pinEncoderB = 4;  //PD4
const char registerMaskPIND = 0b00011000; //Show pin

volatile int32_t motorTickCount = 0;
uint32_t motorTimePrev = 0; //Some of those variables can be ignored/simplify ; Microsec
uint32_t motorTimeNow = 0;
uint32_t motorTimeDiff = 0;

uint32_t motorSpeedRpm = 0;
uint32_t motorSpeedRpmRaw = 0;
uint32_t motorSpeedRpmPrev = 0;
int8_t motorDirection = 0; // {-1;0;1} with 0 => stopped
bool motorAccelerating = false;

/*-----------*/
/*   IMU     */
/*-----------*/

const uint8_t pinIMUInterrupt = 2;
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint32_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint32_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

float ypr[3]; // [yaw, pitch, roll]
uint32_t IMU_time_prev, IMU_time_now; // to compute leanAngle derivative and integer

/*----------------*/
/*   DC MOTOR     */
/*----------------*/

const int8_t pinPotentiometer = 0; // A0
const int8_t pinEscPWM = 11;
const int8_t pinEscDir = 12;
const int16_t motorLimitPwm = 128; //Stay under 6V for 12V alim.
const int16_t motor_Starter_SpeedLimit = 70;
// DC_motor motor( pinEscPWM, pinEscDir, motorLimitPwm);  // PWM = Pin 11, DIR = Pin 12.
int16_t motorCommandPwm = 0;
int16_t motorCommandPwm_prev = 0;
int16_t motorCommandPwm_Offset = 25;
int16_t motorCommandPwm_Starter_Offset = 120;

// =====================================
// ===       ARDUINO PROCESSES       ===
// =====================================

void setup() {
  Wire.begin(); // join I2C bus (I2Cdev library doesn't do this automatically)
  Serial.begin(115200);
  Serial.setTimeout(5);

  Encoder_Setup();
  IMU_setup();

  Serial.println(F("\n\nReady to run !"));
  Serial.print(F("Mode set to ")); Serial.println(machineState);
  timePrev = millis();
}

void loop() {
  if (millis() - timePrev > 1000 / workingFrequency) {

    if (Serial.available()) {
      analogWrite(pinEscPWM, 0); //Shut down motor
      machineState = Serial.read();
      serialViewerMode = machineState;
      Serial.print(F("\n\nState set to ")); Serial.println(machineState);

      if (machineState == '1' | machineState == '3' | machineState == '4') { // reset computing
        mpu.resetFIFO();
        IMU_time_prev = micros();
        IMU_time_now = micros();
        PID_output = 40;
      }
    }

    timePrev = millis();

    switch (machineState) {

      case '1' : // PID controlled mode
        Update_MotorData(); // Update value of motorSpeedRpm (filtered) and motorDirection; both used to compute PIDs
        Update_leanAngle(); // Update the value of ypr[]; ypr[1] is used to compute leanAngle
        leanAngle_compute();
        PID_compute();
        Set_Motor_Speed();
        break;

      case '2' : // Motor test
        Update_MotorData(); // Update value of motorSpeedRpm (filtered) and motorDirection; both used to compute PIDs
        motorCommandPwm = - map(analogRead(A0), 0, 1023, 0, motorLimitPwm);
        Set_Motor_Speed();
        break;

      case '3' : // Setpoint tuning
        Update_MotorData();
        Update_leanAngle(); // Update the value of ypr[]; ypr[1] is used to compute leanAngle
        leanAngle_compute();
        PID_compute();
        break;

      case '4' : // IMU Viewer
      Update_MotorData();
        Update_leanAngle(); // Update the value of ypr[]; ypr[1] is used to compute leanAngle
        leanAngle_compute();
        //PID_compute();
        break;

      case 'p' : // Update KP
        KP = Update_uint32_serial(KP);
        break;

      case 'i' : // Update KI
        KI = Update_uint32_serial(KI);
        break;

      case 'd' : // Update KD
        KD = Update_uint32_serial(KD);
        break;

      case 's' : // Update KS
        KS = Update_uint32_serial(KS);
        break;

      default:
        break;
    }

    computationTime = millis() - timePrev;
    Serial_viewer(serialViewerMode);
  }
  else {
    //Serial.println("_"); //Check working frequency calibration
  }
}
