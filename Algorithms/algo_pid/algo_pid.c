//
// Created by 15082 on 2024/11/21.
//

#include "algo_pid.h"

#include <strings.h>


void algo_pid_init(PID_TypeDef *pid,float Kp, float Ki, float Kd, float maxi, float maxout) {
    pid->kp=Kp;
    pid->ki=Ki;
    pid->kd=Kd;
    pid->max_i=maxi;
    pid->max_out=maxout;
    pid->last1_err=0;
    pid->last2_err=0;
    pid->active =1;
}

float algo_pid_calculate(PID_TypeDef *pid, float set, float real) {
    if(pid->active) {
        pid->Set = set;
        pid->Real = real;
        pid->err = pid->Set - pid->Real;

        pid->last2_err = pid->last1_err;
        pid->last1_err = pid->err;

        pid->Pout = pid->kp * pid->err;
        pid->Dout = pid->kd * (pid->err - 2*pid->last1_err + pid->last2_err);
        pid->Dout = pid->kd * (pid->last1_err - pid->last2_err);
        pid->Iout += pid->err * pid->ki;
        if (pid->Iout > pid->max_i) pid->Iout = pid->max_i;
        else if (pid->Iout < -pid->max_i) pid->Iout = -pid->max_i;

        pid->PIDout = pid->Pout + pid->Iout + pid->Dout;

        if(pid->PIDout >= pid->max_out) pid->PIDout = pid->max_out;
        else if(pid->PIDout <= -pid->max_out) pid->PIDout = -pid->max_out;

        return pid->PIDout;
    }
    else return 0;
}
void algo_pid_clear(PID_TypeDef *pid) {
    bzero(pid,sizeof(PID_TypeDef));
}