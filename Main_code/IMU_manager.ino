/**   SELF STABILIZED BIKE - IMU
 *    
 * note : Working frequency is set in loop(){},
 *        the interrupt pin is therefor not used to trigger data reading.
 *        To avoid delayed transfer, the buffer is emptyed at the end of Update_YPR()
 *        
 * resources : https://www.i2cdevlib.com/docs/html/class_m_p_u6050.html   
 * author : Arthur FINDELAIR, github.com/ArthurFDLR
 * date : September 2019
 */

// ==============================================
// ===               VARIABLES                ===
// ==============================================

MPU6050 mpu;

const int pinIMUInterrupt = 2;  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint32_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint32_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

/*
// ===============================================
// ===       INTERRUPT DETECTION ROUTINE       ===
// ===============================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ISR_dmpDataReady() {
  mpuInterrupt = true;
}
*/
// =========================================================
// ===         FUNCTION IMU : Setup and Update           ===
// =========================================================

void IMU_setup() {
  // initialize device
  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(pinIMUInterrupt, INPUT);

  // verify connection
  // Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

/*
  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
*/
  // load and configure the DMP
  // Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Useless since auto-calibration
  // supply your own gyro offsets here, scaled for min sensitivity
/*
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
*/
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    // Serial.print(digitalPinToInterrupt(pinIMUInterrupt));
    // Serial.println(F(")..."));
    //attachInterrupt(digitalPinToInterrupt(pinIMUInterrupt), ISR_dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    // Serial.print(devStatus);
    // Serial.println(F(")"));
  }
}

void Update_leanAngle() {
  if (!dmpReady) {
    Serial.println("Shit failed yo");
    return;                                               // if programming failed, don't try to do anything
  }
/*
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
    // other program behavior stuff here
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
  }
*/
//  mpuInterrupt = false;                 // reset interrupt flag
  mpuIntStatus = mpu.getIntStatus();    //get INT_STATUS byte
  fifoCount = mpu.getFIFOCount();       // get current FIFO count

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {  // check for overflow (this should never happen unless our code is too inefficient)
    mpu.resetFIFO();        // reset so we can continue cleanly
    Serial.println(F("FIFO overflow!"));
  }

  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {     // otherwise, check for DMP data ready interrupt (this should happen frequently)

    while (fifoCount < packetSize) {   // wait for correct available data length, should be a VERY short wait
      delay(1);
      fifoCount = mpu.getFIFOCount();
      mpuIntStatus = mpu.getIntStatus();
    }

    while (fifoCount >= packetSize) { // Lets catch up to NOW
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      //Serial.println(F("IMU catching up!"));
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    IMU_time_prev = IMU_time_now;
    IMU_time_now = micros();
  }
}