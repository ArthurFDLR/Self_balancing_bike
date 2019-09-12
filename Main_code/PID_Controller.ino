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
float leanAngleError = 0;
const float leanAngle_Filter = 0.7;

float leanAngle_derivative_prev;
const float leanAngle_Derivative_Filter = 0.7;

int16_t P_control,I_control,D_control;
const int16_t KP,KI,KD;


// ==============================================
// ===             FUNCTION PID               ===
// ==============================================


void leanAngle_compute(){
  
  leanAngle_prev = leanAngle;
  leanAngle_raw = ypr[1] * 180 / M_PI; // To cut down noises : round((ypr[1] * 180 / M_PI) * angle_Rounding_Value) / angle_Rounding_Value;
  leanAngle = leanAngle_raw * leanAngle_Filter + leanAngle_prev * (1.0 - leanAngle_Filter);
  
  leanAngleError = leanAngle - leanAngleSetPoint;

  leanAngle_Integer += leanAngleError * (IMU_time_now - IMU_time_prev) / 1000000.0; // Might need anti wind-up

  leanAngle_derivative_prev = leanAngle_derivative;
  leanAngle_derivative = ((leanAngle - leanAngle_prev) / ((IMU_time_now - IMU_time_prev) / 1000000.0)) * leanAngle_Derivative_Filter + leanAngle_derivative_prev * (1.0 - leanAngle_Derivative_Filter);
}


void PID_compute(){

  leanAngle_compute();
  
  P_control = KP * leanAngleError;
  I_control = KI * leanAngle_Integer;
  D_control = KD * leanAngle_derivative;
  
}