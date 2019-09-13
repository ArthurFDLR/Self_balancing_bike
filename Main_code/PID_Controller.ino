/**   SELF STABILIZED BIKE - PID
 *    
 * note : Might need anti wind-up, derivative filtering and automatic setpoint adjustment
 * 
 * resources : https://www.i2cdevlib.com/docs/html/class_m_p_u6050.html   
 * author : Arthur FINDELAIR, github.com/ArthurFDLR
 * date : September 2019
 */

// ==============================================
// ===               VARIABLES                ===
// ==============================================

float leanAngle_prev; //degrees
float leanAngle_smoothed;
float leanAngle_smoothed_prev;
float leanAngleError = 0;
const float leanAngle_Filter = 0.9;
const float leanAngle_HeavyFilter = 0.6;

float leanAngleSetPoint_raw = 0;
float setPoint_filter = 0.8;
uint16_t setPoint_KP, setPoint_KD;

float leanAngle_derivative_prev;
float leanAngle_derivative_smoothed;
const float leanAngle_Derivative_Filter = 0.5;

float P_control,I_control,D_control,S_control;
float PID_output;


const float motorFriction = 0;
float PID_friction;
const int16_t KP,KI,KD,KS; // Tuning constants : Proportional, Integral, Derivative, Speed


// ==============================================
// ===             FUNCTION PID               ===
// ==============================================


void leanAngle_compute(){ // Seems to work --> serialViewerMode=1
  
  leanAngle_prev = leanAngle;
  leanAngle_raw = ypr[1] * 180 / M_PI; // To cut down noises : round((ypr[1] * 180 / M_PI) * angle_Rounding_Value) / angle_Rounding_Value;
  leanAngle = leanAngle_raw * leanAngle_Filter + leanAngle_prev * (1.0 - leanAngle_Filter);
  
  leanAngleError = leanAngle - leanAngleSetPoint;

  leanAngle_Integer += leanAngleError * (IMU_time_now - IMU_time_prev) / 1000000.0; // Might need anti wind-up

  leanAngle_derivative_prev = leanAngle_derivative;
  leanAngle_derivative = ((leanAngle - leanAngle_prev) / ((IMU_time_now - IMU_time_prev) / 1000000.0)) * leanAngle_Derivative_Filter + leanAngle_derivative_prev * (1.0 - leanAngle_Derivative_Filter);

  // Automatic setup adjustment
  leanAngle_smoothed_prev = leanAngle_smoothed;
  leanAngle_smoothed = leanAngle * leanAngle_HeavyFilter + leanAngle_smoothed_prev * (1.0 - leanAngle_HeavyFilter);
  leanAngle_derivative_smoothed = (leanAngle_smoothed - leanAngle_smoothed_prev) / ((IMU_time_now - IMU_time_prev) / 1000000.0);

  leanAngleSetPoint_raw = leanAngleSetPoint - setPoint_KP * leanAngleError - leanAngle_derivative_smoothed * setPoint_KD;
  leanAngleSetPoint = leanAngleSetPoint_raw * setPoint_filter + leanAngleSetPoint * (1.0 - setPoint_filter);
}


void PID_compute(){
  
  P_control = KP * leanAngleError;
  I_control = KI * leanAngle_Integer;
  D_control = KD * leanAngle_derivative;
  PID_output = P_control + I_control + D_control;

  PID_friction = motorDirection * motorFriction;

  S_control = KS * motorDirection * (motorSpeedRpm / 60.0);

  motorCommandPwm = round(constrain( PID_output+PID_friction+S_control ,-motorLimitPwm ,motorLimitPwm));

}
