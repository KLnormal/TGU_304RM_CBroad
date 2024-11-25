//
// Created by 15082 on 2024/11/24.
//

#include "app_sys_init.h"

uint8_t init_flag = 0;

void DJI_Motor_init() {
    algo_dji_init(E_CAN1,E_GM6020,5,E_DJI_Current);
    algo_dji_init(E_CAN1,E_GM6020,6,E_DJI_Current);
}

void app_sys_init(void) {
    app_hardware_init();
    DJI_Motor_init();
    algo_normal_pid_init();
    init_flag = 1;
}
