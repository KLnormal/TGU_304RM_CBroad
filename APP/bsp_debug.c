//
// Created by 15082 on 2024/11/15.
//
#include "bsp_debug.h"
#include "bsp_def.h"

extern uint8_t Init_Finish_Flag;

#define DEBUG_SIZE 1024
static  uint8_t uart_temp[DEBUG_SIZE] = {0};

void Debug(uint16_t flag, const char *fmt, ...) {
    if (Init_Finish_Flag == 0) return;
    switch (BSP_DEBUG & flag) {
        case CAN_CALLBACK_CAN1_DEBUG:
        case CAN_CALLBACK_CAN2_DEBUG:
        case CHASSIS_DEBUG:{
            va_list ap;
            va_start(ap, fmt);
            uint16_t len = vsnprintf(uart_temp, DEBUG_SIZE, fmt, ap);
            va_end(ap);
            bsp_uart_send(E_UART_DEBUG, uart_temp, len);
            break;
        }
        default: break;
    }
}
