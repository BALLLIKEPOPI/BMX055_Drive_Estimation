#ifndef _PID_H_
#define _PID_H_

extern struct pid{
  float SetAngle; 
  float err; 
  float err_next; 
  float err_last; 
  float Kp,Ki,Kd; 
}pid;

void PID_init(struct pid *pid);
float PID_realize(float ActualAngle, struct pid *pid);
float Speed_r2c(float speed);

#endif
