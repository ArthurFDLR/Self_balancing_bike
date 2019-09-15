/**   SELF STABILIZED BIKE - IMU

   note : Working frequency is set in loop(){},
          the interrupt pin is therefor not used to trigger data reading.
          To avoid delayed transfer, the buffer is emptyed at the end of Update_YPR()

   resources : https://www.i2cdevlib.com/docs/html/class_m_p_u6050.html
   author : Arthur FINDELAIR, github.com/ArthurFDLR
   date : September 2019
*/

// =========================================================
// ===         FUNCTION IMU : Setup and Update           ===
// =========================================================

void IMU_setup() {

  mpu.initialize();
  pinMode(pinIMUInterrupt, INPUT);

  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-2222);
  mpu.setYAccelOffset(841);
  mpu.setZAccelOffset(1186);
  mpu.setXGyroOffset(-661);
  mpu.setYGyroOffset(-43);
  mpu.setZGyroOffset(-33);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {

    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("IMU Initialization failed"));
  }
}

void Update_leanAngle() {
  if (!dmpReady) {
    Serial.println("IMU Update failed");
    return;                                               // if programming failed, don't try to do anything
  }

  mpuIntStatus = mpu.getIntStatus();    // get INT_STATUS byte
  fifoCount = mpu.getFIFOCount();       // get current FIFO count

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {  //check for overflow
    mpu.resetFIFO();        // reset so we can continue cleanly
    Serial.println(F("FIFO overflow!"));
  }

  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {     // otherwise, check for DMP data ready interrupt

    while (fifoCount < packetSize) {   // wait for correct available data length, should be a VERY short wait
      delay(1);
      fifoCount = mpu.getFIFOCount();
      mpuIntStatus = mpu.getIntStatus();
    }

    while (fifoCount >= packetSize) { // Lets catch up to the last available value
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    mpu.dmpGetQuaternion(&q, fifoBuffer);    // Read data
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    IMU_time_prev = IMU_time_now;
    IMU_time_now = micros();
  }
}
