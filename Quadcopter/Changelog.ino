/* 
 *  Changelog 
 *  2016-21-31 f41ardu - replace IMU.ino by MPU6050.ino, remove libraries folder and use libraries from  
 *  Arduino base installation. Arduino IDE is 1.6.13 
 *  
 *  Use YAWPITCHROLL calculation from MPU6050_6Axis_MotionApps20.h library
 *  Output of IMU is now in degres according the original IMU from Ben. 
 *  Changed YAW in mpu6050 according Ben's original IMU code. 
 *  Due to instabilities changed Serial.begin(38400) from 115200.
 *  Add LED_C class and replaced heartbeat. 
 *  
 */
