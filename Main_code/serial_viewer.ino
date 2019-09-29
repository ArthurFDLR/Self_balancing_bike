/**   SELF STABILIZED BIKE - Viewer

   note : Might need anti wind-up, derivative filtering and automatic setpoint adjustment

   resources : https://www.i2cdevlib.com/docs/html/class_m_p_u6050.html
   author : Arthur FINDELAIR, github.com/ArthurFDLR
   date : September 2019
*/

// ===========================================
// ===          FUNCTION VIEWER            ===
// ===========================================

void Serial_viewer(char mode) {
  switch ( mode )
  {
    case '1': //PID tuning
      Serial.print(leanAngleSetPoint);
      Serial.print("\t");
      Serial.print(leanAngle);
      Serial.print("\t");
      Serial.print(leanAngleError);
      Serial.print("\t\t");
      Serial.print(P_control);
      Serial.print("\t");
      Serial.print(I_control);
      Serial.print("\t");
      Serial.print(D_control);
      Serial.print("\t");
      Serial.print(PID_output);
      Serial.print("\t");
      Serial.println(motorCommandPwm);
      break;

    case '2': //Encoder debug
      Serial.print(motorCommandPwm);
      Serial.print("\t");
      Serial.print(motorDirection);
      Serial.print("\t");
      Serial.print(motorSpeedRpmRaw);
      Serial.print("\t");
      Serial.print(motorSpeedRpm);
      Serial.print("\t");
      Serial.println(motorAccelerating);
      break;


    case '3': //Setpoint tuning
      Serial.print(leanAngle_smoothed);
      Serial.print("\t");
      Serial.print(leanAngle_derivative_smoothed);
      Serial.print("\t\t");
      Serial.print(leanAngleSetPoint);
      Serial.print("\t");
      Serial.println(leanAngleError);
      break;

    case '4': //IMU viewer
      Serial.print(leanAngle_smoothed);
      Serial.print("\t");
      Serial.print(leanAngle_derivative_smoothed);
      Serial.print("\t\t");
      Serial.println(leanAngle_Integer);
      break;

    default:
      break;
  }
}

uint32_t Update_uint32_serial(uint32_t tunedValue) {
  uint32_t valueRead = 0;
  Serial.print(F("Update constante, current value : ")); Serial.println(tunedValue);
  while (!Serial.available()); //Wait for serial comm
  valueRead = Serial.parseInt();
  Serial.print(F("New value : ")); Serial.println(valueRead);
  machineState = '0'; //Back to mode 0
  Serial.print(F("\n\nState set to ")); Serial.println(machineState);
  return valueRead;
}
