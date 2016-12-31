// To support a direkt connected MPU6050 on Arduino some rework is still required
// This code is primarily based on the I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
/*  Hardware setup:
  MPU6050 Breakout --------- Arduino
  3.3V --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND


*/
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project


void mpu_init() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void mpu_update() {


  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    mpuInterrupt = true;
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // get quaternion values in InvenSense Teapot format and store them in q
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    // calculate YAWPITCHROLL
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // from Ben's IMU code
    // print gyroscope values from fifoBuffer
    // ypr[0] = ((fifoBuffer[24] << 8) + fifoBuffer[25]);
/* 
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
*/ 
    angleZ = ypr[0] * 180 / M_PI;
    angleY = ypr[1] * 180 / M_PI;
    angleX = ypr[2] * 180 / M_PI;
    /*
      Angles in the original code from Ben are in degree.
      He use YAW and ROLL from Eulre angels and YAW direct from GYRO??? 
      What does that mean? 
      // Output Euler Angles
      float euler_x = atan2((2 * q_y * q_z) - (2 * q_w * q_x), (2 * q_w * q_w) + (2 * q_z * q_z) - 1); // phi
      float euler_y = -asin((2 * q_x * q_z) + (2 * q_w * q_y));                                        // theta
      //  float euler_z = atan2((2 * q_x * q_y) - (2 * q_w * q_z), (2 * q_w * q_w) + (2 * q_x * q_x) - 1); // psi
      euler_x = euler_x * 180/M_PI; // angle in degrees -180 to +180
      euler_y = euler_y * 180/M_PI; // angle in degrees -180 to +180
      //  euler_z = euler_z * 180/M_PI; // angle in degrees -180 to +180
      volatile byte* Rol_Ptr = (byte*) &euler_x;
      volatile byte* Pit_Ptr = (byte*) &euler_y;
      //  volatile byte* Yaw_Ptr = (byte*) &euler_z;

      // print gyroscope values from fifoBuffer
      float GyroZ = ((fifoBuffer[24] << 8) + fifoBuffer[25]);
      volatile byte* Yaw_Ptr = (byte*) &GyroZ;
    */

  }
}

