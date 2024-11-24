//
// Created by 15082 on 2024/11/24.
//

#include "app_harware_init.h"

void app_hardware_init(void) {
    bsp_uart_init(E_UART_DEBUG,&huart6);
    bsp_can_init(E_CAN1,&hcan1);
    bsp_can_init(E_CAN2,&hcan2);
    bsp_led_init();
}