/**   SELF STABILIZED BIKE - MOTOR

   note : This part of the code used to be a library,
          it eventually is easier to read and debug without encapsulation

   author : Arthur FINDELAIR, github.com/ArthurFDLR
   date : September 2019
*/

void Motor_Setup() {
  pinMode(pinEscPWM, OUTPUT);
  pinMode(pinEscDir, OUTPUT);
  digitalWrite(pinEscPWM, LOW);
  digitalWrite(pinEscDir, LOW);
}

void Set_Motor_Speed() {

  motorCommandPwm = (int)(PID_output);

  // Motor launching
  if ( (abs(motorSpeedRpm) < 30) & (abs(motorCommandPwm) < motorCommandPwm_Offset)) { // & (abs(motorCommandPwm) > 0)
    motorCommandPwm += ((motorCommandPwm > 0) ? 1 : - 1) * motorCommandPwm_Starter_Offset ;
  } else if (abs(motorCommandPwm) > 1 ) {
    motorCommandPwm += ((motorCommandPwm > 0) ? 1 : - 1) * motorCommandPwm_Offset;
  }


  /*
    if (abs(motorCommandPwm) > motorCommandPwm_Offset){
    motorCommandPwm += ((motorCommandPwm > 0) ? 1 : - 1) * motorCommandPwm_Offset;
    }   else if ((abs(motorCommandPwm) > 1) & (abs(motorCommandPwm) < motorCommandPwm_Offset)){
    motorCommandPwm += ((motorCommandPwm > 0) ? 1 : - 1) * ( motorCommandPwm_Offset/2 + motorCommandPwm );
    }

    else if (abs(motorCommandPwm) > 10) {
    motorCommandPwm *= 2 ;
    } else if (abs(motorCommandPwm) > 1){
    motorCommandPwm = 10;
    }
  */

  // Limit motor command
  if (motorCommandPwm > motorLimitPwm) {
    motorCommandPwm = motorLimitPwm;
  } else if (motorCommandPwm < -motorLimitPwm) {
    motorCommandPwm = -motorLimitPwm;
  }

  // Write speed
  if ((abs(motorCommandPwm - motorCommandPwm_prev) > motorCommandPwm_Offset - 20) & (abs(motorSpeedRpm) < 200)) {
    motorCommandPwm = motorCommandPwm * 0.4 + motorCommandPwm_prev * (1.0 - 0.4);
    motorCommandPwm_prev = motorCommandPwm;
  }

  if (motorCommandPwm >= 0) {
    analogWrite(pinEscPWM, motorCommandPwm);
    digitalWrite(pinEscDir, LOW);
  } else {
    analogWrite(pinEscPWM, -motorCommandPwm);
    digitalWrite(pinEscDir, HIGH);
  }

}
