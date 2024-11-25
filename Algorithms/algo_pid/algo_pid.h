//
// Created by 15082 on 2024/11/21.
//

#ifndef ALGO_PID_H
#define ALGO_PID_H

#include "main.h"

typedef struct
{
    float kp,ki,kd;
    float Set,Real;
    float err;
    float last1_err;
    float last2_err;
    float sum_err;
    float max_sum;
    float Pout,Iout,Dout;
    float PIDout;
    float max_out;
    uint8_t active;
}PID_TypeDef;

void algo_pid_init(PID_TypeDef *pid,float Kp, float Ki, float Kd, float maxsum, float maxout);
float algo_pid_calculate(PID_TypeDef *pid, float set, float real);

#endif //ALGO_PID_H
