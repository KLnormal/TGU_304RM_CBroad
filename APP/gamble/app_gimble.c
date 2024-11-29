//
// Created by 15082 on 2024/11/27.
//

#include "app_gimble.h"


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
float set_yaw =0;

float pid_p,pid_d,pid_i;
void gimbal_debug_api(bsp_uart_e e, uint8_t *s, uint16_t l);
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
    bsp_uart_set_callback(E_UART_DEBUG,gimbal_debug_api);
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

//pid1:XXX.XXX,XXX.XXX,XXX.XXX
//修改gimbal_yaw_speed
//pid2:XXX.XXX,XXX.XXX,XXX.XXX
//修改gimbal_yaw_accel
//yaw:+XXX
//5+7+7+7
void gimbal_debug_api(bsp_uart_e e, uint8_t *s, uint16_t l) {
    if (s[0] == 'p' && s[1] == 'i' && s[2] == 'd') {
        pid_p = (s[5]-'0')*100+(s[6]-'0')*10+(s[7]-'0')+(s[9]-'0')*0.1+(s[10]-'0')*0.01+(s[11]-'0')*0.001;
        pid_i = (s[5+8]-'0')*100+(s[6+8]-'0')*10+(s[7+8]-'0')+(s[9+8]-'0')*0.1+(s[10+8]-'0')*0.01+(s[11+8]-'0')*0.001;
        pid_d = (s[5+16]-'0')*100+(s[6+16]-'0')*10+(s[7+16]-'0')+(s[9+16]-'0')*0.1+(s[10+16]-'0')*0.01+(s[11+16]-'0')*0.001;
        bsp_uart_printf(E_UART_DEBUG,"%d,%d,%d\n",pid_p,pid_i,pid_d);
        if(s[3] == '1') {
            algo_pid_clear(&gimbal_yaw_speed);
            algo_pid_init(&gimbal_yaw_speed,pid_p,pid_i,pid_d,1000,20000);
        }
        else if(s[3] == '2') {
            algo_pid_clear(&gimbal_yaw_accel);
            algo_pid_init(&gimbal_yaw_accel,pid_p,pid_i,pid_d,1000,20000);
        }
    }
    if (s[0] == 'y' && s[1] == 'a' && s[2] == 'w') {
        set_yaw = (s[5]-'0')*100 + (s[6]- '0')*10 + (s[7] - '0');
        if(s[4] == '-') set_yaw = -set_yaw;
        bsp_uart_printf(E_UART_DEBUG,"%d\n",set_yaw);
    }
}