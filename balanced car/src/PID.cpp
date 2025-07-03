#include <PID.h>

void PID_Init(PID_Param* pid, float Kp, float Ki, float Kd, float max_out, float min_out)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->output = 0;

    pid->output_max = max_out;
    pid->output_min = min_out;
}


void PID_Calculate(PID_Param * pid, float measured)
{
    pid->measured = measured;
    pid->err = pid->target - pid->measured;
    pid->integral += pid->err;

    pid->output = pid->Kp * pid->err
            + pid->Ki * pid->integral
            + pid->Kd * (pid->err - pid->err_last);

    if (pid->output > pid->output_max) pid->output = pid->output_max;
    else if (pid->output < pid->output_min) pid->output = pid->output_min;

    pid->err_last = pid->err;
}
