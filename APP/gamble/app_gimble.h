//
// Created by 15082 on 2024/11/27.
//

#ifndef APP_GAMBLE_H
#define APP_GAMBLE_H

#include "algo_pid.h"
#include "algo_pid.h"
#include "algo_dji.h"
#include "bsp_imu.h"
#include "bsp_uart.h"

void app_gimbal_init();
float app_gimbal_sum(float set_yaw);

#endif //APP_GAMBLE_H
