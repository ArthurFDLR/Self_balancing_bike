/**   SELF STABILIZED BIKE - PID
 *    
 * note : Might need anti wind-up, derivative filtering and automatic setpoint adjustment
 *
 * author : Arthur FINDELAIR, github.com/ArthurFDLR
 * date : September 2019
 */

// ==============================================
// ===             FUNCTION PID               ===
// ==============================================

void leanAngle_compute(){ // Seems to work --> serialViewerMode=1
  
  leanAngle_prev = leanAngle;
  leanAngle_raw = ypr[1] * 180 / M_PI; // To cut down noises : round((ypr[1] * 180 / M_PI) * angle_Rounding_Value) / angle_Rounding_Value;
  leanAngle = leanAngle_raw * leanAngle_Filter + leanAngle_prev * (1.0 - leanAngle_Filter);
  
  leanAngleError = leanAngleSetPoint - leanAngle;

  leanAngle_Integer += leanAngleError * (IMU_time_now - IMU_time_prev) / 1000000.0; // Might need anti wind-up

  leanAngle_derivative_prev = leanAngle_derivative;
  leanAngle_derivative = (float) ((leanAngle - leanAngle_prev) / ((IMU_time_now - IMU_time_prev) / 1000000.0)) * leanAngle_Derivative_Filter + leanAngle_derivative_prev * (1.0 - leanAngle_Derivative_Filter);

  // Automatic setup adjustment
  leanAngle_smoothed_prev = leanAngle_smoothed;
  leanAngle_smoothed = leanAngle * leanAngle_HeavyFilter + leanAngle_smoothed_prev * (1.0 - leanAngle_HeavyFilter);
  leanAngle_derivative_smoothed_prev = leanAngle_derivative_smoothed;
  leanAngle_derivative_smoothed = leanAngle_derivative * leanAngle_HeavyFilter + leanAngle_derivative_smoothed_prev * (1.0 - leanAngle_HeavyFilter);

  leanAngleSetPoint_prev = leanAngleSetPoint;
  //leanAngleSetPoint -= (float) (setPoint_KP * leanAngleError - leanAngle_derivative_smoothed * setPoint_KD) / 10000.0;
  //leanAngleSetPoint = leanAngleSetPoint_raw * setPoint_filter + leanAngleSetPoint * (1.0 - setPoint_filter);
}


void PID_compute(){
  
  P_control = KP * leanAngleError;
  I_control = KI * leanAngle_Integer;
  D_control = KD * leanAngle_derivative;

  PID_friction = motorDirection * motorFriction;

  S_control = KS * motorDirection * (motorSpeedRpm / 60.0);

  PID_output = 40 + (P_control + I_control + D_control + PID_friction + S_control) / 100;

  if (PID_output > motorLimitPwm) {
    PID_output = motorLimitPwm;
  } else if (PID_output < -motorLimitPwm) {
    PID_output = -motorLimitPwm;
  }

  
  
  //motorCommandPwm = (int)(PID_output + PID_friction + S_control);
  //motorCommandPwm = ((motorCommandPwm > motorLimitPwm) ? motorLimitPwm : motorCommandPwm);
  //motorCommandPwm = ((motorCommandPwm < -motorLimitPwm) ? -motorLimitPwm : motorCommandPwm);
  //motorCommandPwm = round(constrain( (int)(PID_output + PID_friction + S_control) ,-motorLimitPwm ,motorLimitPwm));

}
