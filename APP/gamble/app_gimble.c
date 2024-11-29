//
// Created by 15082 on 2024/11/27.
//

#include "app_gimble.h"

#include "algo_pid.h"
#include "algo_dji.h"
#include "bsp_imu.h"

#define K_degree 3
#define K_accel 2
/*#define K_forward 0.02579*/
#define K_correct 1.2

//K_forward 取决于转动惯量

//云台前馈参数: 转动惯量J 前馈电压:deltaU 前馈电压系数K 角加速度α
//本代码前馈电压系数按照物理公式和电机的电压与电流的函数拟合而成，在6020上我们发现电压和电流的关系大致为线性关系
//

//下面的几个define是和电机以及云台的参数有关
#define GIMBAL_INE 19923.793
#define PI 3.14159265358979323846
#define MOTOR_PRECISION 16384.0
#define DIMENSION 1000000.0
#define MAX_I 3.0
#define VOLTAGE_TO_I 0.8
#define I_TO_MOMENT 0.6
#define DEGREE_TO_RADIAN 180.0/PI
float K_forward;

PID_TypeDef gimbal_yaw_speed;
PID_TypeDef gimbal_yaw_accel;

float set_speed,set_accel,forward;
float set_voltage;


void forward_calculate() {
    K_forward = VOLTAGE_TO_I;//把电压转化为电流的反馈量（MAX16348）
    K_forward = K_forward*MAX_I/MOTOR_PRECISION;
    K_forward = K_forward * I_TO_MOMENT;
    K_forward = K_forward * DEGREE_TO_RADIAN *DIMENSION;
    K_forward = K_forward / GIMBAL_INE;
}
void app_gimbal_init() {
    algo_pid_init(&gimbal_yaw_speed,10,0.5,0,1000,20000);
    algo_pid_init(&gimbal_yaw_accel,2,0.3,0,1000,20000);
    forward_calculate();
}

float feed_forward(float set_accel) {
    float feed_forward = 0;
    feed_forward = set_accel * K_correct/ K_forward ;
    return feed_forward;
}

float accel_control(float set_accel) {
    float accel_out= 0;
    accel_out = algo_pid_calculate(&gimbal_yaw_accel,set_accel,imu_get(E_IMU_Accel)[E_Accel_Yaw]);
    accel_out += feed_forward(set_accel);
    return accel_out ;
}

float app_gimbal_sum(float set_yaw) {
    set_speed = (set_yaw - ins.yaw) * K_degree;
    set_accel = (set_speed - imu_get(E_IMU_Gyro)[E_Gyro_Yaw]) * K_accel;
    set_voltage = accel_control(set_accel) + algo_pid_calculate(&gimbal_yaw_speed,set_speed,algo_dji_get(E_CAN1,E_GM6020,5).real_speed);
    return set_voltage;
}


void uart1_call_back() {}