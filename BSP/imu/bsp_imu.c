//
// Created by fish on 2024/9/18.
//

#include "bsp_imu.h"
#include "BMI088driver.h"

static bsp_imu_raw_data_t raw_data;

bsp_imu_raw_data_t *bsp_imu_read() {
    BMI088_read(raw_data.gyro, raw_data.accel, &raw_data.temp);
    return &raw_data;
}

void bsp_imu_init() {
    while(BMI088_init());
}
