// https://oscarliang.com/quadcopter-pid-explained-tuning/



PID roll_controller(&pid_roll_in,   &pid_roll_out,  &pid_roll_setpoint,  5.0, 0.0, 0.0, REVERSE);
PID pitch_controller(&pid_pitch_in, &pid_pitch_out, &pid_pitch_setpoint, 5.0, 0.0, 0.0, REVERSE);
PID yaw_controller(&pid_yaw_in,     &pid_yaw_out,   &pid_yaw_setpoint,   1.0, 0.0, 0.0, DIRECT); 


void pid_initialize() {
  roll_controller.SetOutputLimits(ROLL_PID_MIN,ROLL_PID_MAX);
  pitch_controller.SetOutputLimits(PITCH_PID_MIN,PITCH_PID_MAX);
  yaw_controller.SetOutputLimits(YAW_PID_MIN,YAW_PID_MAX);
  roll_controller.SetMode(AUTOMATIC);
  pitch_controller.SetMode(AUTOMATIC);
  yaw_controller.SetMode(AUTOMATIC);
  roll_controller.SetSampleTime(10);
  pitch_controller.SetSampleTime(10);
  yaw_controller.SetSampleTime(10);
}

void pid_update(){
/*  angleZ = ypr[0] * 180 / M_PI; // YAW
    angleY = ypr[1] * 180 / M_PI; // PITCH
    angleX = ypr[2] * 180 / M_PI; // ROLL
 */   
  pid_roll_in = angles[2];   // angleX
  pid_pitch_in = angles[1];  // angleY
  pid_yaw_in = angles[0];    // angleZ 
}

void pid_compute() {
   roll_controller.Compute();
   pitch_controller.Compute();
   yaw_controller.Compute();
}
