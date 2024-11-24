//
// Created by 15082 on 2024/11/21.
//

#include "algo_pid.h"


void algo_pid_init(PID_TypeDef *pid,float Kp, float Ki, float Kd,float imax, float maxsum, float maxout)
{
    pid->kp=Kp;
    pid->ki=Ki;
    pid->kd=Kd;
    pid->imax=imax;
    pid->max_sum=maxsum;
    pid->max_out=maxout;
    pid->last1_err=0;
    pid->last2_err=0;
    pid->active =1;
}

float algo_pid_calculate(PID_TypeDef *pid, float set, float real)
{
    if(pid->active)
    {
        pid->Set = set;
        pid->Real = real;
        pid->err = pid->Set - pid->Real;
        pid->sum_err = pid->sum_err + pid->err;

        if(pid->sum_err >= pid->max_sum) pid->sum_err = pid->max_sum;
        else if(pid->sum_err <= -pid->max_sum) pid->sum_err = -pid->max_sum;

        pid->last2_err = pid->last1_err;
        pid->last1_err = pid->err;

        pid->Pout = pid->kp * pid->err;
        pid->Iout = pid->ki * pid->sum_err;
        pid->Dout = pid->kd * (pid->err - 2*pid->last1_err + pid->last2_err);
        pid->PIDout = pid->Pout + pid->Iout + pid->Dout;

        if(pid->PIDout >= pid->max_out) pid->PIDout = pid->max_out;
        else if(pid->PIDout <= -pid->max_out) pid->PIDout = -pid->max_out;

        return pid->PIDout;
    }
    else
        return 0;
}