//
// Created by fish on 2024/9/18.
//

#ifndef BSP_IMU_H
#define BSP_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gyro[3], accel[3], temp;
} bsp_imu_raw_data_t;

void bsp_imu_init();
bsp_imu_raw_data_t *bsp_imu_read();

#ifdef __cplusplus
}
#endif

#endif //BSP_IMU_H
