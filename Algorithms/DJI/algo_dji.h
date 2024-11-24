//
// Created by 15082 on 2024/11/24.
//

#ifndef ALGO_DJI_H
#define ALGO_DJI_H

#include "algo_motor_def.h"


void algo_dji_init(bsp_can_e can_e, dji_motor_type_e motor_e, uint8_t id, dji_ctr_mode_e ctr);
void algo_dji_set(bsp_can_e can_e, dji_motor_type_e motor_e, uint8_t id, int16_t data);
void algo_dji_send(bsp_can_e can_e, uint16_t stdid);
typedef_dji_motor algo_dji_get(bsp_can_e can_e, dji_motor_type_e motor_e, uint8_t id);

#endif //ALGO_DJI_H
