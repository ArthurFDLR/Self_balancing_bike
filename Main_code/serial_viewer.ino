/**   SELF STABILIZED BIKE - Viewer
 *    
 * note : Might need anti wind-up, derivative filtering and automatic setpoint adjustment
 *        
 * resources : https://www.i2cdevlib.com/docs/html/class_m_p_u6050.html   
 * author : Arthur FINDELAIR, github.com/ArthurFDLR
 * date : September 2019
 */

// ===========================================
// ===          FUNCTION VIEWER            ===
// ===========================================


void Serial_viewer(uint8_t mode) {
  switch ( mode )
  {
    case 1:
      Serial.print(leanAngle_raw);
      Serial.print("\t");
      Serial.print(leanAngle);
      Serial.print("\t");
      Serial.print(leanAngle_Integer);
      Serial.print("\t");
      Serial.println(leanAngle_derivative);
      break;
    
    case 2:
      Serial.print(motorCommandPwm);
      Serial.print("\t");
      Serial.print(motorDirection);
      Serial.print("\t");
      Serial.print(motorSpeedRpmRaw);
      Serial.print("\t");
      Serial.println(motorSpeedRpm);
      break;
    
    default:
      break;
  }

}
