//
// Created by 15082 on 2024/11/29.
//

#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "algo_pid.h"
#include "algo_dji.h"
#include "algo_motor_def.h"
#include "bsp_rc.h"

void app_chassis_init();
void app_chassis_control(float w, float x, float y);

#endif //APP_CHASSIS_H
