//
// Created by 15082 on 2024/11/24.
//

#include "algo_dji.h"

#include "bsp_def.h"

#define MOTOR_COUNT 6
#define CAN_PASSAGE 2
#define PACKAGE_COUNT 8

uint8_t motor_count=0;
typedef_dji_motor DJI_motor[MOTOR_COUNT];

//这些代码不对报文ID和电机ID冲突负责，如果ID冲突建议动一动自己金贵的小手指动一下拨码开关
//电机的Stdid和占用的标志位之类的全都不能冲突，这个自己调整，这份代码能使用的前提是你电机的ID都配置正确

void motor_id_calculate(dji_motor_type_e motor_e, uint8_t id,typedef_dji_motor *motor_ary) {
    uint16_t std_id;
    uint16_t feedback_id;
    if (motor_e == E_M3508 || motor_e == E_M2006) {
        std_id = 0x200 - (id-1)/4;
        feedback_id = 0x200 + id;
        motor_ary->feedback_id = feedback_id;
        motor_ary->std_id = std_id;
    }
    else if (motor_e == E_GM6020 && motor_ary->ctr_mode == E_DJI_Current) {
        std_id = 0x1FE + 0X100*(id-1)/4;
        feedback_id = 0x204 + 0X100*id;
        motor_ary->feedback_id = feedback_id;
        motor_ary->std_id = std_id;
    }
    else if (motor_e == E_GM6020 && motor_ary->ctr_mode == E_DJI_Voltage) {
        std_id = 0x1FF + 0X100*(id-1)/4;
        feedback_id = 0x204 + 0X100*id;
        motor_ary->feedback_id = feedback_id;
        motor_ary->std_id = std_id;
    }
}
void dji_can1_callback(bsp_can_msg_t *msg) {
    uint16_t feedback_id =msg->header.StdId;
    uint8_t ary_flag = 0;
    while ( DJI_motor[ary_flag].feedback_id != feedback_id && DJI_motor[ary_flag].can_handle== E_CAN1) {
        ary_flag++;
        if (ary_flag >= MOTOR_COUNT) return;
    }
    DJI_motor[ary_flag].real_position = msg->data[0]<<8 | msg->data[1];
    DJI_motor[ary_flag].real_speed =  msg->data[2]<<8 | msg->data[3];
    DJI_motor[ary_flag].real_current = msg->data[4]<<8 | msg->data[5];
    DJI_motor[ary_flag].real_temp = msg->data[6];
}
void dji_can2_callback(bsp_can_msg_t *msg) {
    uint16_t feedback_id =msg->header.StdId;
    uint8_t ary_flag = 0;
    while ( DJI_motor[ary_flag].feedback_id != feedback_id && DJI_motor[ary_flag].can_handle== E_CAN2) {
        ary_flag++;
        if (ary_flag >= MOTOR_COUNT) return;
    }
    DJI_motor[ary_flag].real_position = msg->data[0]<<8 | msg->data[1];
    DJI_motor[ary_flag].real_speed =  msg->data[2]<<8 | msg->data[3];
    DJI_motor[ary_flag].real_current = msg->data[4]<<8 | msg->data[5];
    DJI_motor[ary_flag].real_temp = msg->data[6];
}

uint8_t can_ary[CAN_PASSAGE][PACKAGE_COUNT];

void algo_dji_init(bsp_can_e can_e, dji_motor_type_e motor_e, uint8_t id, dji_ctr_mode_e ctr) {
    BSP_ASSERT(id < MOTOR_COUNT);
    DJI_motor[motor_count].can_handle = can_e;
    DJI_motor[motor_count].motor_type = motor_e;
    DJI_motor[motor_count].motor_id = id;
    DJI_motor[motor_count].ctr_mode = ctr;
    motor_id_calculate(motor_e, id, &DJI_motor[motor_count]);
    if (can_e == E_CAN1) bsp_can_set_callback(E_CAN1, DJI_motor[motor_count].feedback_id, dji_can1_callback);
    else if (can_e == E_CAN2)bsp_can_set_callback(E_CAN2, DJI_motor[motor_count].feedback_id, dji_can2_callback);
    motor_count++;
}

void algo_dji_set(bsp_can_e can_e, dji_motor_type_e motor_e, uint8_t id, int16_t data) {
    uint8_t flag = 0;
    while (flag < MOTOR_COUNT) {
        if (DJI_motor[flag].can_handle == E_CAN1 && DJI_motor[flag].motor_type == motor_e && DJI_motor[flag].motor_id == id) {
            DJI_motor[flag].set_data = data;
            return;
        }
        flag++;
    }
    BSP_ASSERT(0);
}

void algo_dji_send(bsp_can_e can_e, uint16_t stdid) {
    uint8_t flag = 0;
    uint8_t ary_flag = 0;
    while (flag < MOTOR_COUNT) {
        if (DJI_motor[flag].std_id == stdid && DJI_motor[flag].can_handle == can_e) {
            ary_flag =  (DJI_motor[flag].motor_id -1)%4;
            can_ary[can_e][ary_flag*2] = DJI_motor[flag].set_data >> 8;
            can_ary[can_e][ary_flag*2+1] = DJI_motor[flag].set_data;
        }
        flag++;
    }
    bsp_can_send(can_e,stdid,can_ary[can_e]);
}