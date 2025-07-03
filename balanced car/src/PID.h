#ifndef _PID_H
#define _PID_H

typedef struct 
{
    float Kp, Ki, Kd;
    float target;
    float measured;
    float err;
    float err_last;
    float integral;

    float output;
    float output_max;
    float output_min;
}PID_Param;

void PID_Init(PID_Param* pid, float Kp, float Ki, float Kd, float max_out, float min_out);
void PID_Calculate(PID_Param * pid, float measured);

#endif