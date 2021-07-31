/**
  ******************************************************************************
  * 文件名          : AttitudeResolve.c
  * 文件描述        : 陀螺仪数据解析
  * 创建时间        : 2021.1.10
  * 作者            : 颜剑燊
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.1.10
  * 修改人          : 颜剑燊
  ******************************************************************************
  * 1.本代码基于STM32F407IGH6TR开发，编译环境为Keil 5，基于FreeRTOS进行开发。
  * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
	* 3.本代码包含大量中文注释，请以UTF-8编码格式打开
	* 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
  ******************************************************************************
  */
	
#include "BMI088Middleware.h"
#include "tim.h"

extern SPI_HandleTypeDef hspi1;

void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}

//只能在采用systick作为时钟源时使用
void BMI088_delay_us(uint16_t us)
{

//    uint32_t ticks = 0;
//    uint32_t told = 0;
//    uint32_t tnow = 0;
//    uint32_t tcnt = 0;
//    uint32_t reload = 0;
//    reload = SysTick->LOAD;
//    ticks = us * 168;
//    told = SysTick->VAL;
//    while (1)
//    {
//        tnow = SysTick->VAL;
//        if (tnow != told)
//        {
//            if (tnow < told)
//            {
//                tcnt += told - tnow;
//            }
//            else
//            {
//                tcnt += reload - tnow + told;
//            }
//            told = tnow;
//            if (tcnt >= ticks)
//            {
//                break;
//            }
//        }
//    }

	uint16_t counter=0xffff-us-5;
	HAL_TIM_Base_Start(&htim14);
	__HAL_TIM_SetCounter(&htim14,counter);
	while(counter<0xffff-5)
	{
		counter=__HAL_TIM_GetCounter(&htim14);
	}
	HAL_TIM_Base_Stop(&htim14);

}

//片选信号选中加速度计
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
//取消对加速度计的片选信号
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

