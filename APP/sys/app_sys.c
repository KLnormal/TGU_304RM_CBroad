//
// Created by 15082 on 2024/11/22.
//

#include "app_sys.h"

void app_sys_init(void) {
    bsp_led_init();
    bsp_led_set(255,255,255);
}

uint8_t led_light =255;
int16_t direct = 1;

void app_sys_loop(void) {
    if (led_light == 255) direct = -direct;
    else if (led_light == 0) direct = -direct;
    led_light += direct;
    bsp_led_set(led_light,led_light,led_light);
}