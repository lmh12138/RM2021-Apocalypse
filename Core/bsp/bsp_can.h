#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"

#define CAN_ID_MOTOR1 0x201
#define CAN_ID_MOTOR2 0x202
#define CAN_ID_MOTOR3 0x203
#define CAN_ID_MOTOR4 0x204
#define CAN_ID_MOTOR5 0x205
#define CAN_ID_MOTOR6 0x206
#define CAN_ID_MOTOR7 0x207
#define CAN_ID_MOTOR9 0x209

extern HAL_StatusTypeDef CanFilterInit(CAN_HandleTypeDef* hcan);
//extern void AddDatapackToTxQueue(CAN_HandleTypeDef* hcan, CAN_DATAPACK_t* txDataPack);

void Can1DataReceive(int motor_index, uint8_t *RecvData);
void Can2DataReceive(int motor_index, uint8_t *RecvData);

#endif
