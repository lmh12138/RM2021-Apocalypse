/**
  ******************************************************************************
  * 文件名          : pc_communicate.c
  * 创建时间        : 2020.01.01
  * 作者            : 张大明
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.05.9
  * 修改人          : 方纬博
  ******************************************************************************
  * 1.本代码基于STM32F407IGH6TR开发，编译环境为Keil 5，基于FreeRTOS进行开发。
  * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
	* 3.本代码包含大量中文注释，请以UTF-8编码格式打开
	* 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
  ******************************************************************************
**/

#include "pc_communicate.h"
#include "AttitudeResolve.h"
#include "robot.h"
#include "supervise.h"
#include "datatypes.h"
#include "string.h"

#define RX_LEN 16
#define TX_LEN 11

//
extern struct Robot_t infantry;
	
SendData data_rx;
McuData data_tx;

uint8_t pc_tx[TX_LEN] = {0};  
uint8_t pc_rx[RX_LEN] = {0};

uint8_t Vision_Get_New_Data = 0;

void PC_Communicate_init(void){
	__HAL_UART_ENABLE_IT(&PC_huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&PC_huart, pc_rx, RX_LEN);
}
void PC_IDLECallback(void) {
    HAL_UART_DMAStop(&PC_huart);
    uint8_t data_length = RX_LEN - __HAL_DMA_GET_COUNTER(PC_huart.hdmarx);
    PC_DataHandler(data_length);
	Vision_Get_New_Data = 1;//标记视觉数据更新了
    HAL_UART_Receive_DMA(&PC_huart, pc_rx, RX_LEN);
}

void PC_UART_IRQHandler(UART_HandleTypeDef* huart) {
    if (huart == &PC_huart) {
        if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            PC_IDLECallback();
        }
    }
}

int Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}
/**
  * @brief  视觉数据更新标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = 0;
}

/**
	* @brief 接收上位机数据
	* @param none
	* @retval none
	*/
void PC_DataHandler(uint8_t data_length)
{
	if(data_length != RX_LEN) return;
	if(pc_rx[0] != 's' || pc_rx[RX_LEN - 1] != 'e') return;
	memcpy(&data_rx,pc_rx,RX_LEN);
	Vision_Get_New_Data = 1;//标记视觉数据更新了
	static float z_last_none_zero = 0;
	static int8_t zero_cnt = 0;
	if(data_rx.z != 0){
		z_last_none_zero = data_rx.z;
		zero_cnt = 0;
	}
	else if(zero_cnt < 10){
		zero_cnt ++;
		data_rx.z = z_last_none_zero;
	}
	else
		data_rx.z = 0;
	LostCounterFeed(PC_INDEX);
}

/**
	* @brief 发送接口
	* @param none
	* @retval none
	*/
void PcDataTramsmit(enum Data_Type_t data_type)
{
	float SpeedLimit = (float)robot_state.shooter_id1_17mm_speed_limit;
	if(SpeedLimit != 15 && SpeedLimit != 18 && SpeedLimit != 30)SpeedLimit = 15;
	uint8_t RobotColor;
	RobotColor = (robot_state.robot_id >= 100) ? 1: 0;
	
	if(data_type==Speed)
		data_tx = generateSpeedMcuData(SpeedLimit);      //传入正确数据
	else if(data_type==Pan)
		data_tx = generatePanMcuData(-bmi088.gyro.z * 57.29578f, -bmi088.gyro.x * 57.29578f);
	else if(data_type==Energy)
		data_tx = generateEnergyMcuData(0, 0);
	else if(data_type==Config)
	{
		if(Remote.key.g == 1 && Remote.key.ctrl == 0)
		data_tx = generateConfigMcuData('s', 0, RobotColor);
		else if(Remote.key.b == 1 && Remote.key.ctrl == 0)
		data_tx = generateConfigMcuData('b', 0, RobotColor);
		else
		data_tx = generateConfigMcuData('a', 0, RobotColor);
	}	
	memcpy(pc_tx, &data_tx, TX_LEN);
	HAL_UART_Transmit_DMA(&PC_huart,(uint8_t *)pc_tx,TX_LEN);
}
