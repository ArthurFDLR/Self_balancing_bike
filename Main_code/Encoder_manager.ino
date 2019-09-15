/**   SELF STABILIZED BIKE - Encoder

   note : I tried to build a library to read the encoder.
          ISR calling inside of class is quite complicate.
          I didn't wanted to risk a lose of effectiveness.

   author : Arthur FINDELAIR, github.com/ArthurFDLR
   date : September 2019
*/

// ==================================================================
// ===         FUNCTION ENCODER : ISR, Setup and Update           ===
// ==================================================================

void ISR_updateEncoder() { //Direct access to register to optimize time consumption
  if (((registerMaskPIND & PIND) == 0b00010000) or ((registerMaskPIND & PIND) == 0b00001000)) {
    motorTickCount++;
  } else if (((registerMaskPIND & PIND) == 0b00000000) or ((registerMaskPIND & PIND) == 0b00011000)) {
    motorTickCount--;
  }
}

void Encoder_Setup() {
  pinMode(pinEncoderAInterrupt, INPUT_PULLUP);
  pinMode(pinEncoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncoderAInterrupt), ISR_updateEncoder, CHANGE);

  motorTimePrev = micros();

  motorTickCount = 0;

}


void Update_MotorData() {

  //Get motor rotation speed from tick count
  motorTimePrev = motorTimeNow;
  motorTimeNow = micros();

  motorTimeDiff = (motorTimeNow - motorTimePrev); //micros
  
  motorSpeedRpmPrev = motorSpeedRpm;
  motorSpeedRpmRaw = ((motorTickCount > 0) ? motorTickCount : -motorTickCount) * (60000000.0 / (encoderCountRev * motorTimeDiff)); // 60000000 => micros to min     / (encoderCountRev * motorTimeDiff)

  //Set motor rotation direction
  if (abs(motorTickCount) > 1){
    motorDirection = ((motorTickCount > 0) ? 1 : -1);
  } else {
    motorDirection = 0;
  }
  
  motorTickCount = 0;
  //Filtering
  motorSpeedRpm = motorSpeedRpmRaw * motorFilter + motorSpeedRpmPrev * (1.0 - motorFilter);
}
