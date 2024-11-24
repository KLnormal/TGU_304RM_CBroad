//
// Created by fish on 2024/9/2.
//

#pragma once

#include "stdint.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "bsp_led.h"

#ifdef __cplusplus
extern "C" {
#endif

__attribute__((unused)) static void bsp_assert_err(const char* file, uint32_t line) {
    UNUSED(file); UNUSED(line);
    // while(1) bsp_uart_printf(E_UART_DEBUG, "BSP Assert error at %s:%lu\r\n", file, line);
    bsp_led_set(255, 0, 0);
    while(1) __NOP();
}
#define BSP_ASSERT(arg) if(!(arg)) bsp_assert_err(__FILE__, __LINE__);

typedef enum {
    BSP_OK = 0,
    BSP_ERR = 1,
} bsp_status_t;

#ifdef __cplusplus
}
#endif
