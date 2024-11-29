//
// Created by 15082 on 2024/11/29.
//

#include "app_chassis.h"

#define K_X 1.8
#define K_Y 1.8
#define K_W 2

PID_TypeDef ChassisPID_ID1;
PID_TypeDef ChassisPID_ID2;
PID_TypeDef ChassisPID_ID3;
PID_TypeDef ChassisPID_ID4;

int16_t Chassis_Speed_ID1;
int16_t Chassis_Speed_ID2;
int16_t Chassis_Speed_ID3;
int16_t Chassis_Speed_ID4;

void app_chassis_init() {
    algo_pid_init(&ChassisPID_ID1,10,0.3,0,2000,10000);
    algo_pid_init(&ChassisPID_ID2,10,0.3,0,2000,10000);
    algo_pid_init(&ChassisPID_ID3,10,0.3,0,2000,10000);
    algo_pid_init(&ChassisPID_ID4,10,0.3,0,2000,10000);
    bsp_rc_init();
}
//逆时针为正
void app_chassis_control(float w, float x, float y) {
    Chassis_Speed_ID1 = y*K_Y + x*K_X + w*K_W;
    Chassis_Speed_ID4 = y*K_X - x*K_W + w*K_W;
    Chassis_Speed_ID2 = -(y*K_X - x*K_W - w*K_W);
    Chassis_Speed_ID3 = -(y*K_X + x*K_W - w*K_W);
    int16_t temp =0;
    temp = algo_pid_calculate(&ChassisPID_ID1,Chassis_Speed_ID1,algo_dji_get(E_CAN2,E_M3508,1).real_speed);
    algo_dji_set(E_CAN2,E_M3508,1,temp);
    temp = algo_pid_calculate(&ChassisPID_ID2,Chassis_Speed_ID2,algo_dji_get(E_CAN2,E_M3508,2).real_speed);
    algo_dji_set(E_CAN2,E_M3508,2,temp);
    temp = algo_pid_calculate(&ChassisPID_ID3,Chassis_Speed_ID3,algo_dji_get(E_CAN2,E_M3508,3).real_speed);
    algo_dji_set(E_CAN2,E_M3508,3,temp);
    temp = algo_pid_calculate(&ChassisPID_ID4,Chassis_Speed_ID4,algo_dji_get(E_CAN2,E_M3508,4).real_speed);
    algo_dji_set(E_CAN2,E_M3508,4,temp);
    algo_dji_send(E_CAN2,0x200);
}