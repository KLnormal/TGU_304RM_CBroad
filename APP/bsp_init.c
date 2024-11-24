//
// Created by 15082 on 2024/11/18.
//

#include "bsp_init.h"


uint8_t Init_Finish_Flag = 0;

void can_init(void) {
    bsp_can_init(E_CAN1,&hcan1);
    bsp_can_init(E_CAN2,&hcan2);
}

void led_init(void) {
    bsp_led_init();
    bsp_led_set(0,0,255);
}

void app_hardware_init(void) {
    led_init();
    bsp_led_set(0,0,255);
    can_init();
    bsp_uart_init(E_UART_DEBUG,&huart6);
    bsp_rc_init();
    HAL_Delay(3000);
    bsp_led_set(0,255,0);
    Init_Finish_Flag = 1;
}