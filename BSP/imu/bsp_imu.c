//
// Created by fish on 2024/4/27.
//

#include "bsp_imu.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "algo_pid.h"
#include "tim.h"
#include "bsp_uart.h"

#define correct_times 1000
#define temp_times 50

PID_TypeDef Temperature_PID;
float Temperature_PID_Para[3] = {7500,100,0 };

static float gyro[3], accel[3], gyro_correct[3];
//gyro: roll, pitch, yaw 定义站在电源口看向另一边为正，roll:向左滚转为正 pitch:上抬机头为正 yaw:向左偏航为正
//规定站在CAN口看向SWD烧录口的方向为Y轴正方向
static float imu_temp;
static uint32_t correct_cnt, temp_cnt;

bsp_imu_ins ins;
float ins_degree_accel[3];
uint8_t imuInit() {
    IMU_QuaternionEKF_Init(10, 0.001f, 10000000, 1, 0.001f, 0);
    algo_pid_init(&Temperature_PID,7000,100,1,100,10000);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    while(BMI088_init());
    return 1;
}

void imuTemperatureCtrl() {
    float out = (uint32_t) algo_pid_calculate(&Temperature_PID, 41.0f, imu_temp);    // 温控 40 度
    if(out < 0) out = 0;
    htim10.Instance->CCR1 = out;
}

static uint8_t ins_flag = 0;
static uint32_t count = 0;
static float imuTemp_yaw,imuTemp_roll,imuTemp_pitch;
/*
 * 供 rtos 调用的 task，用于更新 imu 数据，1khz
 */
void imuTask() {
    BMI088_read(gyro, accel, &imu_temp);
    if(ins_flag == 2) {
        /* 减去陀螺仪零飘 */
        gyro[0] -= gyro_correct[0];
        gyro[1] -= gyro_correct[1];
        gyro[2] -= gyro_correct[2];

        /* yaw 轴小幅度转动视为零飘，这里的阈值取决于实际情况 */
#ifdef CHEAT
        if(fabsf(gyro[2]) < 0.01f) gyro[2] = 0.0f;
#endif

        IMU_QuaternionEKF_Update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);

        ins_degree_accel[E_Accel_Yaw] = (gyro[E_Gyro_Yaw] - imuTemp_yaw)*1000;
        ins_degree_accel[E_Accel_Pitch] = (gyro[E_Gyro_Pitch] - imuTemp_pitch)*1000;
        ins_degree_accel[E_Accel_Roll] = (gyro[E_Gyro_Roll] - imuTemp_roll)*1000;
        imuTemp_yaw = gyro[E_Gyro_Yaw];
        imuTemp_roll = gyro[E_Gyro_Roll];
        imuTemp_pitch = gyro[E_Gyro_Pitch];

        ins.yaw = Get_Yaw();
        ins.roll = Get_Roll();
        ins.pitch = Get_Pitch();

    } else if (ins_flag == 1) {
        gyro_correct[0] += gyro[0];
        gyro_correct[1] += gyro[1];
        gyro_correct[2] += gyro[2];
        correct_cnt++;
        if(correct_cnt == correct_times) {
            gyro_correct[0] /= correct_times;
            gyro_correct[1] /= correct_times;
            gyro_correct[2] /= correct_times;
            ins_flag = 2;
        }
    }
    if(count % 10 == 0) {
        // 100hz
        imuTemperatureCtrl();
        if(fabsf(imu_temp - 40) < 0.5f && ins_flag == 0) {
            temp_cnt++;
            if(temp_cnt == temp_times) {
                ins_flag = 1;
            }
        }
    }
    count ++;
    /*bsp_uart_printf(E_UART_DEBUG, "imu temperature: %.2f\n", imu_temp);*/
    /*msgVofaSend(E_UART1, 8, (float) ins_flag, imu_temp, ins.roll, ins.pitch, ins.yaw, gyro_correct[0], gyro_correct[1], gyro_correct[2]);*/
}

uint8_t imuReady() {
    return ins_flag == 2;
}

float* imu_get(imu_e e) {
    if(e == E_IMU_Accel) return ins_degree_accel;
    else if(e == E_IMU_Gyro) return gyro;
    else if (e== E_IMU_Gyro_correct) return gyro_correct;
    return NULL;
}
