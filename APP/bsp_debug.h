//
// Created by 15082 on 2024/11/14.
//

//该代码实现了只需要配置BSP_DEBUG即可完成配置DEBUG的模式，在需要预留DEBUG接口的地方写Debug(BSP_DEBUG, "debug_data")即可完成debug

#ifndef BSP_DEBUG_DEFINE_H
#define BSP_DEBUG_DEFINE_H

#include "main.h"
#include "bsp_uart.h"
#include "bsp_def.h"
#include "stdio.h"

#define CAN_CALLBACK_CAN1_DEBUG (0b0001)
#define CAN_CALLBACK_CAN2_DEBUG (0b0001<<1)
#define CHASSIS_DEBUG (0b0001<<2)
#define BSP_DEBUG (CHASSIS_DEBUG)

void Debug(uint16_t flag, const char *fmt, ...);

#endif //BSP_DEBUG_DEFINE_H
