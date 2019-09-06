void Serial_viewer() {
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(motorPwm);
    Serial.print("\t");
    Serial.print(motorSpeedRpm);
    Serial.print("\t");
    Serial.println(computationTime);
}
