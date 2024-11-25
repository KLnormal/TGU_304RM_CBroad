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
    E_Gyro,
    E_Accel,
    E_Gyro_correct
}imu_e;
uint8_t imuInit();
void imuTask();
uint8_t imuReady();
float* imu_get(imu_e e);

extern bsp_imu_ins ins;

#endif //DM_MC_02_FRAMEWORK_BSP_IMU_H
