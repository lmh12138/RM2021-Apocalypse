/**
  ******************************************************************************
  * 文件名          : remote.c
  * 创建时间        : 2019.12.28
  * 作者            : 邓紫龙
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2019.12.28
  * 修改人          : 邓紫龙
  ******************************************************************************
  * 1.本代码基于STM32F407IGH6TR开发，编译环境为Keil 5，基于FreeRTOS进行开发。
  * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
	* 3.本代码包含大量中文注释，请以UTF-8编码格式打开
	* 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
  ******************************************************************************
  */
  
#include "remote.h"
#include "supervise.h"
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
 
uint8_t rc_data[18];          //遥控器接收的原始数据
struct DT7Remote_t Remote;  	//遥控器数据结构体

/**
	* @brief 串口初始化函数
	* @param None
	* @retval None
	*/
void Remote_init(void)
{
	__HAL_UART_ENABLE_IT(&huart_Remote, UART_IT_IDLE);//设置中断为空闲中断模式
  HAL_UART_Receive_DMA(&huart_Remote, rc_data, 18u);
}

/**
	* @brief 监控弱定义函数
	* @param 电机宏定义号
	* @retval 无
	*/
__weak void LostCounterFeed_Remote(void){}
	
/**
	* @brief 串口空闲中断（中断回调）函数
	* @param 串口号
	* @retval None
	*/
void REMOTE_IDLE_Callback(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))      //判断一帧数据是否接收完毕
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);                 //清空IDLE标志位
		USART_Remote->SR;                                	//清空SR寄存器
		USART_Remote->DR;                                 //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(huart, DMA_FLAG_TCIF_Remote); 	//清空DMA传输完成标志位
		HAL_UART_DMAStop(huart);
		if(__HAL_DMA_GET_COUNTER(huart->hdmarx) == 0)     //如果接收的长度是18
		{
			RemoteData();//数据解析
			LostCounterFeed(RC_INDEX);
		}
		HAL_UART_Receive_DMA(huart,rc_data,18u);		      //再次使能接收，NDTR重载
	}
}

/**
	* @brief 遥控器数据具体解析函数
	* @param None
	* @retval None
	*/
void RemoteData()
{
	Remote.last_rc = Remote.rc;
	Remote.last_keyboard = Remote.keyboard;
	Remote.last_mouse = Remote.mouse;
	
	Remote.rc.ch0 = ((int16_t)rc_data[0] | ((int16_t)rc_data[1] << 8)) & 0x07FF; 
	Remote.rc.ch1 = (((int16_t)rc_data[1] >> 3) | ((int16_t)rc_data[2] << 5)) & 0x07FF;
	Remote.rc.ch2 = (((int16_t)rc_data[2] >> 6) | ((int16_t)rc_data[3] << 2) | ((int16_t)rc_data[4] << 10)) & 0x07FF;
	Remote.rc.ch3 = (((int16_t)rc_data[4] >> 1) | ((int16_t)rc_data[5]<<7)) & 0x07FF;
	Remote.rc.ch4 = ((int16_t)rc_data[16] | ((int16_t)rc_data[17] << 8)) & 0x07FF;
	
	Remote.rc.s1 = ((rc_data[5] >> 4) & 0x000C) >> 2;
	Remote.rc.s2 = ((rc_data[5] >> 4) & 0x0003);

	Remote.mouse.x = ((int16_t)rc_data[6]) | ((int16_t)rc_data[7] << 8);
	Remote.mouse.y = ((int16_t)rc_data[8]) | ((int16_t)rc_data[9] << 8);
	Remote.mouse.z = ((int16_t)rc_data[10]) | ((int16_t)rc_data[11] << 8);
	
	Remote.mouse.press_l = rc_data[12];
	Remote.mouse.press_r = rc_data[13];

	Remote.keyboard = ((int16_t)rc_data[14])  | ((int16_t)rc_data[15] << 8);

  /*根据遥控器s1/s2档位改变机器人状态,默认s2=2时为stop*/
	if(Remote.rc.s2 == 2)
		Remote.inputmode = RC_Stop;
	else if(Remote.rc.s2 == 3)
		Remote.inputmode = RC_Remote;
	else if(Remote.rc.s2 == 1)
		Remote.inputmode = RC_MouseKey;
	//改善操作体验(Pitch的范围比Yaw小，容易受扰动)
//	if(Remote.mouse.x > 5 || Remote.mouse.x < -5)	
//		Remote.mouse.y = 0;

	KeyStateChange();
}

void KeyStateChange(void)
{
	if(Remote.inputmode != RC_Stop)
	{
		if((Remote.keyboard&Key_W)==Key_W && (Remote.last_keyboard&Key_W)==0)Remote.key.w = 1-Remote.key.w;
		if((Remote.keyboard&Key_S)==Key_S && (Remote.last_keyboard&Key_S)==0)Remote.key.s = 1-Remote.key.s;
		if((Remote.keyboard&Key_D)==Key_D && (Remote.last_keyboard&Key_D)==0)Remote.key.d = 1-Remote.key.d;
		if((Remote.keyboard&Key_A)==Key_A && (Remote.last_keyboard&Key_A)==0)Remote.key.a = 1-Remote.key.a;
		if((Remote.keyboard&Key_Shift)==Key_Shift && (Remote.last_keyboard&Key_Shift)==0)Remote.key.shift = 1-Remote.key.shift;
		if((Remote.keyboard&Key_ECtrl)==Key_ECtrl && (Remote.last_keyboard&Key_ECtrl)==0)Remote.key.ctrl = 1-Remote.key.ctrl;
		if((Remote.keyboard&Key_Q)==Key_Q && (Remote.last_keyboard&Key_Q)==0)Remote.key.q = 1-Remote.key.q;
		if((Remote.keyboard&Key_E)==Key_E && (Remote.last_keyboard&Key_E)==0)Remote.key.e = 1-Remote.key.e;
		if((Remote.keyboard&Key_R)==Key_R && (Remote.last_keyboard&Key_R)==0)Remote.key.r = 1-Remote.key.r;
		if((Remote.keyboard&Key_F)==Key_F && (Remote.last_keyboard&Key_F)==0)Remote.key.f = 1-Remote.key.f;
		if((Remote.keyboard&Key_G)==Key_G && (Remote.last_keyboard&Key_G)==0)Remote.key.g = 1-Remote.key.g;
		if((Remote.keyboard&Key_Z)==Key_Z && (Remote.last_keyboard&Key_Z)==0)Remote.key.z = 1-Remote.key.z;
		if((Remote.keyboard&Key_X)==Key_X && (Remote.last_keyboard&Key_X)==0)Remote.key.x = 1-Remote.key.x;
		if((Remote.keyboard&Key_C)==Key_C && (Remote.last_keyboard&Key_C)==0)Remote.key.c = 1-Remote.key.c;
		if((Remote.keyboard&Key_V)==Key_V && (Remote.last_keyboard&Key_V)==0)Remote.key.v = 1-Remote.key.v;
		if((Remote.keyboard&Key_B)==Key_B && (Remote.last_keyboard&Key_B)==0)Remote.key.b = 1-Remote.key.b;
	}
	
	static int reset_cnt;
	if((Remote.keyboard&Key_ECtrl) && (Remote.keyboard&Key_G) && Remote.rc.s2 == 2)//ctrl+g 软重启功能 
	{
		reset_cnt++;
		if(reset_cnt >= 70)
			soft_reset();
	}
	else
		reset_cnt = 0;
}

void soft_reset()
{
	__set_PRIMASK(1);
	HAL_NVIC_SystemReset();
}
