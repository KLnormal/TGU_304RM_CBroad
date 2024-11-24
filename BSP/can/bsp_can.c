//
// Created by fish on 2024/11/15.
//

#include "bsp_can.h"

#include "bsp_def.h"

static uint8_t tot;
static uint8_t cnt[E_CAN_END_DONT_REMOVE];

static CAN_HandleTypeDef *handle[E_CAN_END_DONT_REMOVE];

static uint32_t rx_id[E_CAN_END_DONT_REMOVE][BSP_CAN_FILTER_LIMIT];
static void (*callback[E_CAN_END_DONT_REMOVE][BSP_CAN_FILTER_LIMIT]) (bsp_can_msg_t *msg);

void bsp_can_init(bsp_can_e e, CAN_HandleTypeDef *h) {
    handle[e] = h;
    HAL_CAN_Start(h);
    // HAL_CAN_ActivateNotification(h, CAN_IT_TX_MAILBOX_EMPTY);
    HAL_CAN_ActivateNotification(h, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(h, CAN_IT_RX_FIFO1_MSG_PENDING);
}

uint8_t can_filter_idx[2] = { 0, 14 };

uint8_t bsp_can_set_callback(bsp_can_e e, uint32_t id, void (*f) (bsp_can_msg_t *msg)) {
    BSP_ASSERT(tot < BSP_CAN_FILTER_LIMIT && f != NULL);
    rx_id[e][cnt[e]] = id;
    callback[e][cnt[e]] = f;

    // 前 14 个过滤器分给 CAN1，后 14 个分给 CAN2，因此每个 CAN 最多带 14 个设备。
    // 参考代码：https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/can/bsp_can.c
    CAN_FilterTypeDef filter = {
        .FilterMode = CAN_FILTERMODE_IDLIST,
        .FilterScale = CAN_FILTERSCALE_16BIT,
        .FilterFIFOAssignment = (id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1,
        .SlaveStartFilterBank = 14,
        .FilterIdLow = id << 5,
        .FilterBank = handle[e] == &hcan1 ? (can_filter_idx[0] ++) : (can_filter_idx[1] ++),
        .FilterActivation = CAN_FILTER_ENABLE
    };

    BSP_ASSERT(HAL_CAN_ConfigFilter(handle[e], &filter) == HAL_OK);
    return tot ++, cnt[e] ++;
}

void bsp_can_send(bsp_can_e e, uint32_t id, uint8_t *s) {
    BSP_ASSERT(handle[e]);
    CAN_TxHeaderTypeDef header = {
        .StdId = id,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 0x08
    };
    while(HAL_CAN_GetTxMailboxesFreeLevel(handle[e]) == 0) __NOP(); // 等待邮箱空闲
    uint32_t tx_mailbox = 0;
    HAL_CAN_AddTxMessage(handle[e], &header, s, &tx_mailbox);
}

void bsp_can_rx_sol(bsp_can_e e, uint32_t fifo) {
    bsp_can_msg_t msg = { .port = e };
    while(HAL_CAN_GetRxFifoFillLevel(handle[e], fifo)) {
        HAL_CAN_GetRxMessage(handle[e], fifo, &msg.header, msg.data);
        for(uint8_t i = 0; i < cnt[e]; i++) {
            if(rx_id[e][i] == msg.header.StdId) {
                BSP_ASSERT(callback[e][i] != NULL);
                callback[e][i](&msg);
            }
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h) {
    for(uint8_t i = 0; i < (uint8_t) E_CAN_END_DONT_REMOVE; i++) {
        if(handle[i] == h) bsp_can_rx_sol(i, CAN_RX_FIFO0);
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *h) {
    for(uint8_t i = 0; i < (uint8_t) E_CAN_END_DONT_REMOVE; i++) {
        if(handle[i] == h) bsp_can_rx_sol(i, CAN_RX_FIFO1);
    }
}