//
// Created by fish on 2024/4/27.
//

#ifndef DM_MC_02_FRAMEWORK_BSP_IMU_H
#define DM_MC_02_FRAMEWORK_BSP_IMU_H

#include "BMI088driver.h"

/* 通过忽略小范围变动的方法减小零飘，会导致精度降低带来误差 */
#define CHEAT

typedef struct {
    float pitch, roll, yaw;
} bsp_imu_ins;
typedef enum {
    E_IMU_Gyro,
    E_IMU_Accel,
    E_IMU_Gyro_correct
}imu_e;

typedef enum {
    E_Gyro_Roll,
    E_Gyro_Pitch,
    E_Gyro_Yaw
}imu_gyro_e;
typedef enum {
    E_Accel_X,
    E_Accel_Y,
    E_Accel_Z
}imu_accel_e;
typedef enum {
    E_Accel_Yaw,
    E_Accel_Roll,
    E_Accel_Pitch
}imu_degree_e;


uint8_t imuInit();
void imuTask();
uint8_t imuReady();
float* imu_get(imu_e e);

extern bsp_imu_ins ins;
extern float ins_degree_accel[3];
#endif //DM_MC_02_FRAMEWORK_BSP_IMU_H
