#include "pid.h"
#include "math.h"

void PID_init(struct pid *pid)
{
       pid->SetAngle = 0.0;
       pid->err = 0.0;
       pid->err_last = 0.0;
       pid->err_next = 0.0;
       pid->Kp = 0.4;
       pid->Ki = 0.035;
       pid->Kd = 0.2;
}

float PID_realize(float ActualAngle, struct pid *pid)
{
       float  SetSpeed;

       pid->err = pid->SetAngle - ActualAngle;
       SetSpeed = pid->Kp * (pid->err - pid->err_next) + pid->Ki * pid->err + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last);
       pid->err_last = pid->err_next;
       pid->err_next = pid->err;
       
       return SetSpeed;
}

float Speed_r2c(float speed)
{
       //72M = 12800 * 10 * n * compare
       float compare;
       speed = fabs(speed);       
       compare = 72000000/(128000 * speed);
       return compare;
}
