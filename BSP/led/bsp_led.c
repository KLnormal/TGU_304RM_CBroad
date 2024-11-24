//
// Created by fish on 2024/9/2.
//

#include "bsp_led.h"
#include "bsp_def.h"
#include "tim.h"

void bsp_led_init(void) {
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void bsp_led_set(uint8_t r, uint8_t g, uint8_t b) {
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, b);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, g);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, r);
}