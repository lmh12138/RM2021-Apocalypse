#include "bsp_can.h"
#include "motor.h"
#include "super_cap.h"
#include "supervise.h"

//使用CAN总线的外设
extern struct CAN_Motor can1_motor_1;
extern struct CAN_Motor can1_motor_2;
extern struct CAN_Motor can1_motor_3;
extern struct CAN_Motor can1_motor_4;
extern struct CAN_Motor can1_motor_5;
extern struct CAN_Motor can1_motor_6;
extern struct CAN_Motor can1_motor_7;
extern struct CAN_Motor can1_motor_9;

extern struct CAN_Motor can2_motor_1;
extern struct CAN_Motor can2_motor_2;
extern struct CAN_Motor can2_motor_3;
extern struct CAN_Motor can2_motor_4;
extern struct CAN_Motor can2_motor_5;
extern struct CAN_Motor can2_motor_6;
extern struct CAN_Motor can2_motor_7;
extern struct CAN_Motor can2_motor_9;

//接收数组
uint8_t CanRecvData_FIFO0[8];
uint8_t CanRecvData_FIFO1[8];

//超级电容
__weak void SuperCap_Encoder(struct SuperCap_t *cap, uint8_t *RecvData)
{
}

/**
	* @brief CAN外设过滤器初始化
	* @param can结构体
	* @retval None
	*/
HAL_StatusTypeDef CanFilterInit(CAN_HandleTypeDef* hcan)
{
	// IdHigh, IdLow, MaskIdHigh, MaskIdLow 填写FIFO过滤的四个电机ID, 有空余填写0x800
	// FIFO0
	CAN_FilterTypeDef  sFilterConfig;
	if(hcan == &hcan1)
		sFilterConfig.FilterBank = 0;
	if(hcan == &hcan2)
		sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = 0x201 << 5;
	sFilterConfig.FilterIdLow = 0x202 << 5;
	sFilterConfig.FilterMaskIdHigh = 0x203 << 5;
	sFilterConfig.FilterMaskIdLow = 0x204 << 5;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	
	if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	// FIFO1
	if(hcan == &hcan1)
    {
        sFilterConfig.FilterBank = 1;
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
        sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
        sFilterConfig.FilterIdHigh = 0x205 << 5;
        sFilterConfig.FilterIdLow = 0x206 << 5;
        sFilterConfig.FilterMaskIdHigh = 0x207 << 5;
        sFilterConfig.FilterMaskIdLow = 0x8FF << 5;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.SlaveStartFilterBank = 14;
    }
	if(hcan == &hcan2)
    {
        sFilterConfig.FilterBank = 15;
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
        sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
        sFilterConfig.FilterIdHigh = 0x8FF << 5;
        sFilterConfig.FilterIdLow = 0x8FF << 5;
        sFilterConfig.FilterMaskIdHigh = 0x209 << 5;
        sFilterConfig.FilterMaskIdLow = 0x211 << 5;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.SlaveStartFilterBank = 14;

    }
	
	if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	
	if (HAL_CAN_Start(hcan) != HAL_OK)
	{
		Error_Handler();
	}
	// FIFO0 IT
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}
	// FIFO1 IT
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}
	return HAL_OK;
}

//__weak void Can1DataReceive(int motor_index, uint8_t *RecvData)
//{
//	
//}

//__weak void Can2DataReceive(int motor_index, uint8_t *RecvData)
//{
//	
//}


/**
	* @brief CAN通信接收中断回调函数-FIFO0
	* @param CAN序号
	* @retval None
	*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CanRecvData_FIFO0) != HAL_OK)
	{
		Error_Handler();            //如果CAN通信数据接收出错，则进入死循环
	}
	if(hcan == &hcan1)
		Can1DataReceive(RxHeader.StdId, CanRecvData_FIFO0);   //进行CAN数据解析
	else if(hcan == &hcan2)
		Can2DataReceive(RxHeader.StdId, CanRecvData_FIFO0);
}
/**
	* @brief CAN通信接收中断回调函数-FIFO1
	* @param CAN序号
	* @retval None
	*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, CanRecvData_FIFO1) != HAL_OK)
	{
		Error_Handler();            //如果CAN通信数据接收出错，则进入死循环
	}
	if(hcan == &hcan1)
		Can1DataReceive(RxHeader.StdId, CanRecvData_FIFO1);   //进行CAN数据解析
	else if(hcan == &hcan2)
		Can2DataReceive(RxHeader.StdId, CanRecvData_FIFO1);
}

/**
	* @brief 根据电机信息的ID号进行对应的数据解析-CAN1
	* @param 电机ID号
	* @retval None
	*/
void Can1DataReceive(int motor_index, uint8_t *RecvData)
{
	switch(motor_index)
	{
	case CAN_ID_MOTOR1:
		LostCounterFeed(CAN_MOTOR1_INDEX);
		MotorEncoderProcess(&can1_motor_1, RecvData);break;    //电机数据具体解析函数
	case CAN_ID_MOTOR2:
		LostCounterFeed(CAN_MOTOR2_INDEX);
		MotorEncoderProcess(&can1_motor_2, RecvData);break;
	case CAN_ID_MOTOR3:
		LostCounterFeed(CAN_MOTOR3_INDEX);
		MotorEncoderProcess(&can1_motor_3, RecvData);break;
	case CAN_ID_MOTOR4:
		LostCounterFeed(CAN_MOTOR4_INDEX);
		MotorEncoderProcess(&can1_motor_4, RecvData);break;
	case CAN_ID_MOTOR5:
		LostCounterFeed(CAN_MOTOR5_INDEX);
		MotorEncoderProcess(&can1_motor_5, RecvData);break;
	case CAN_ID_MOTOR6:
		LostCounterFeed(CAN_MOTOR6_INDEX);
		MotorEncoderProcess(&can1_motor_6, RecvData);break;
	case CAN_ID_MOTOR7:
		LostCounterFeed(CAN_MOTOR7_INDEX);
		MotorEncoderProcess(&can1_motor_7, RecvData);break;
	case CAN_ID_MOTOR9:
		LostCounterFeed(CAN_MOTOR9_INDEX);
		MotorEncoderProcess(&can1_motor_9, RecvData);break;
	case SUPER_CAP_ID:
		LostCounterFeed(SUPER_CAP_INDEX);
		SuperCap_Encoder(&spuer_cap, RecvData);break;
	default:;
	}
}

/**
	* @brief 根据电机信息的ID号进行对应的数据解析-CAN2
	* @param 电机ID号
	* @retval None
	*/
void Can2DataReceive(int motor_index, uint8_t *RecvData)
{
	switch(motor_index)
	{
	case CAN_ID_MOTOR1:
		LostCounterFeed(CAN_MOTOR1_INDEX);
		MotorEncoderProcess(&can2_motor_1, RecvData);break;    //电机数据具体解析函数
	case CAN_ID_MOTOR2:
		LostCounterFeed(CAN_MOTOR2_INDEX);
		MotorEncoderProcess(&can2_motor_2, RecvData);break;
	case CAN_ID_MOTOR3:
		LostCounterFeed(CAN_MOTOR3_INDEX);
		MotorEncoderProcess(&can2_motor_3, RecvData);break;
	case CAN_ID_MOTOR4:
		LostCounterFeed(CAN_MOTOR4_INDEX);
		MotorEncoderProcess(&can2_motor_4, RecvData);break;
	case CAN_ID_MOTOR5:
		LostCounterFeed(CAN_MOTOR5_INDEX);
		MotorEncoderProcess(&can2_motor_5, RecvData);break;
	case CAN_ID_MOTOR6:
		LostCounterFeed(CAN_MOTOR6_INDEX);
		MotorEncoderProcess(&can2_motor_6, RecvData);break;
	case CAN_ID_MOTOR7:
		LostCounterFeed(CAN_MOTOR7_INDEX);
		MotorEncoderProcess(&can2_motor_7, RecvData);break;
	case CAN_ID_MOTOR9:
		LostCounterFeed(CAN_MOTOR9_INDEX);
		MotorEncoderProcess(&can2_motor_9, RecvData);break;
	case SUPER_CAP_ID:
		LostCounterFeed(SUPER_CAP_INDEX);
	  SuperCap_Encoder(&spuer_cap, RecvData);break;
	default:;
	}
}

/**
	* @brief CAN通信发送排队函数
	* @param CAN序号,数据包
	* @retval None
	*/
//void AddDatapackToTxQueue(CAN_HandleTypeDef* hcan, CAN_DATAPACK_t* txDataPack)
//{
//	if(HAL_CAN_AddTxMessage(hcan,&(txDataPack->TxHeader),txDataPack->TxData,(uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
//	{
////		HAL_CAN_AbortTxRequest(hcan,(uint32_t)CAN_TX_MAILBOX0);
////		HAL_CAN_AbortTxRequest(hcan,(uint32_t)CAN_TX_MAILBOX1);
////		HAL_CAN_AbortTxRequest(hcan,(uint32_t)CAN_TX_MAILBOX2);
////		while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == RESET);
//	}
//}
