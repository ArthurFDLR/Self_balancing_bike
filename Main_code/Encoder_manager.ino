/**   SELF STABILIZED BIKE - Encoder
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
const int8_t encoderCountRev = 212;
const int8_t pinEncoderAInterrupt = 3; //PD3
const int8_t pinEncoderB = 4;  //PD4

const char registerMaskPIND = 0b00011000; //Show pin 
volatile int32_t motorTickCount = 0;
uint32_t motorTimePrev = 0; //Some of those variables can be ignored/simplify ; Microsec
uint32_t motorTimeNow = 0;
uint32_t motorTimeDiff = 0;
uint32_t motorSpeedRpmPrev = 0;

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
  
  motorTimePrev = micros();

  motorTickCount = 0;
}


void Update_MotorData(){

  //Get motor rotation speed from tick count
  motorTimeNow = micros();
  motorTimeDiff = motorTimeNow - motorTimePrev;
  motorTimePrev = motorTimeNow;
  motorSpeedRpmPrev = motorSpeedRpm;
  motorSpeedRpmRaw = ((motorTickCount > 0) ? motorTickCount : -motorTickCount) * 60000000 / (encoderCountRev * motorTimeDiff); // 60000000 => micros to min
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
