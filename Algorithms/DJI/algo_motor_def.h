//
// Created by 15082 on 2024/11/22.
//

#ifndef ALGO_MOTOR_DEF_H
#define ALGO_MOTOR_DEF_H

#include "main.h"
#include "bsp_can.h"

typedef enum {
    E_M3508,
    E_GM6020,
    E_M2006,
    E_Motor_Types
}dji_motor_type_e;
typedef enum {
    E_DJI_Low,
    E_DJI_High
}dji_motor_data_bits_e;
typedef enum {
    E_DJI_Current,
    E_DJI_Voltage
}dji_ctr_mode_e;


typedef struct {
    bsp_can_e can_handle;
    dji_motor_type_e motor_type;
    dji_ctr_mode_e ctr_mode;
    uint16_t std_id;
    uint16_t feedback_id;
    uint8_t motor_id;
    int16_t set_data;
    int16_t real_speed;
    int16_t real_temp;
    int16_t real_current;
    int16_t real_position;
}typedef_dji_motor;

#endif //ALGO_MOTOR_DEF_H
