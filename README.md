# Quadcopter #

### Arduino Quadcopter Flight Controller ###
Just an other Quadrocopter code. Based on Ben Ripley's code. But instead of starting from scratch I use Ben's code as a seed. 
And due to lack of an ARDUIMU I decided to integrate the IMU (MPU6050) directly into the system. 

#### Operation ####
Software prototying, no hardeware available.

#### External libraries ####
As far as possible I plan to make use of already available libraries. All Required external libraries have been included in the libraries folder of this repository. 
See [Arduino libraries](http://arduino.cc/en/Guide/Libraries), Current libraries used

jrowberg/i2cdevlib: MPU6050_6Axis_MotionApps20.h
http://playground.arduino.cc/Code/PIDLibrary PID_v1.h
http://playground.arduino.cc/Main/PinChangeInt PinChangeInt.h 


#### Flight Controller Board ####
I use an Arduino Duemilanove for devleopment.  
Tunig is required for the components used. 

#### IMU Board ####
MPU6050 (GY-521)

#### More Info ####
Stay tuned. 