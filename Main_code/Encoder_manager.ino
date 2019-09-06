/**
 * Run on arduino nano to controle and measure DC motor's speed.
 *
 * note : I tried to build a library to read the encoder.
 *        ISR calling inside of class is quite complicate.
 *        I didn't wanted to risk a lose of effectiveness.
 *        
 * author : Arthur FINDELAIR, github.com/ArthurFDLR
 * date : September 2019
 */

/*---------------------------------------*/
/*   VARIABLE for Encoder management     */
/*---------------------------------------*/

const float motorFilter = 0.5;       // [0,1] exponential filter parameter
const int encoderCountRev = 212;
const int pinEncoderAInterrupt = 3; //PD3
const int pinEncoderB = 4;  //PD4

const char registerMaskPIND = 0b00011000; //Show pin 
volatile long motorTickCount = 0;
unsigned long motorTimePrev = 0; //Some of those variables can be ignored/simplify
unsigned long motorTimeNow = 0;
unsigned long motorTimeDiff = 0;
unsigned long motorSpeedRpmPrev = 0;
unsigned long motorSpeedRpmRaw = 0;


/*---------------------------------------*/
/*   FUNCITON for Encoder management     */
/*---------------------------------------*/


void ISR_updateEncoder(){ //Direct access to register to optimize time consumption
  if (((registerMaskPIND & PIND) == 0b00010000) or ((registerMaskPIND & PIND) == 0b00001000)){ 
    motorTickCount++;
  } else if (((registerMaskPIND & PIND) == 0b00000000) or ((registerMaskPIND & PIND) == 0b00011000)){
    motorTickCount--;
  }
}

void Encoder_Setup(){
  pinMode(pinEncoderAInterrupt, INPUT_PULLUP);
  pinMode(pinEncoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncoderAInterrupt), ISR_updateEncoder, CHANGE);
  
  motorTimePrev = millis();

  motorTickCount = 0;
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
