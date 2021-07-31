/**
  ******************************************************************************
  * 文件名          : shoot_interface.c
  * 文件描述        : 步兵机器人发射系统交互控制
  * 创建时间        : 2020.7.31
  * 作者            : 邓紫龙
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.3.19
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

#include "shoot.h"
#include "robot.h"
#include "datatypes.h"

extern struct Robot_t infantry;
extern SendData data_rx;

int cur_pos = 0, expect_pos = 0, delta_pos = 0;
uint16_t last_data_u = 0;

/**
	* @brief 射击控制-Remote
	* @param None
	* @retval None
	*/
void ShootCtrl_Remote(void)
{
	ShootCtrl_SpeedLimit_17mm();
	
	uint16_t CoolingLimit = robot_state.shooter_id1_17mm_cooling_limit;
	uint16_t heat0 = power_heat_data_t.shooter_id1_17mm_cooling_heat;
	if(heat0 < CoolingLimit - 16)
    {
		infantry.shoot.FireRate = 3.0f*(float)(Remote.rc.ch4 - CHx_BIAS);
        infantry.shoot.fireRateLeft = 0;
        infantry.shoot.fireRateRight = 0;
    }
	else
    {
		infantry.shoot.FireRate = 0;
        infantry.shoot.fireRateLeft = 0;
        infantry.shoot.fireRateRight = 0;
    }
	
	//弹舱盖
	if(infantry.ShootMode == shoot_disabled)
	{
		static int last_ch4, cur_ch4;
		last_ch4 = cur_ch4; cur_ch4 = Remote.rc.ch4;
		if(cur_ch4 > 1424 && last_ch4 < 1424)
			TIM1->CCR1 = 1790;
		else if(cur_ch4 < 624 && last_ch4 > 624)
			TIM1->CCR1 = 840;
	}
}

/**
	* @brief 射击控制-MouseKey
	* @param None
	* @retval None
	*/
void ShootCtrl_MouseKey(void)
{
	ShootCtrl_SpeedLimit_17mm();
	
	
	//static int shootway_cnt = 0;
	//if(Remote.mouse.press_r == 1 && Remote.last_mouse.press_r == 0)   //右键切换射击模式  //
	//{
		if(infantry.GameMode == AtkBuffActivate)
			{
				infantry.ShootWay = SingleShoot;
				delta_pos = 36864;
			}
		else {infantry.ShootWay = ContinuousShoot;}	
		/*
		if(infantry.GameMode != AtkBuffActivate)
			return;
		shootway_cnt ++;
		if(shootway_cnt > 3)
			shootway_cnt = 0;
		switch(shootway_cnt)
		{
		case 0: infantry.ShootWay = ContinuousShoot;break;
		case 1: infantry.ShootWay = SingleShoot;delta_pos = 49146;break;
		case 2: infantry.ShootWay = DoubleShoot;delta_pos = 98292;break;
		case 3: infantry.ShootWay = TribleShoot;delta_pos = 147438;break;
		default: infantry.ShootWay = ContinuousShoot;
		}
		*/
	//}
	
	if(infantry.ShootWay == ContinuousShoot) //速度环
	{
        infantry.shoot.FireRate = 0;
		uint16_t CoolingLimit = robot_state.shooter_id1_17mm_cooling_limit;
		uint16_t heat0 = power_heat_data_t.shooter_id1_17mm_cooling_heat;
        uint16_t heat1 = power_heat_data_t.shooter_id2_17mm_cooling_heat;
		if(Remote.mouse.press_l == 1 && heat1 < CoolingLimit - 15)	//17mm 拨弹速度调节-防止超热量
		{
            infantry.shoot.fireRateLeft = 2600;
		}
		else
			infantry.shoot.fireRateLeft = 0;
        if(Remote.mouse.press_r == 1 && heat0 < CoolingLimit - 15)	//17mm 拨弹速度调节-防止超热量
		{
			infantry.shoot.fireRateRight = 2600;	//9.62
		}
		else
			infantry.shoot.fireRateRight = 0;
	}
	else //位置环
	{
		infantry.shoot.FireRate = 2600;
        infantry.shoot.fireRateRight = 0;
        infantry.shoot.fireRateLeft = 0;
		cur_pos = SHOOT_PLUCK_MOTOR1.real_position;
		if((Remote.mouse.press_l == 1 && Remote.last_mouse.press_l == 0)|| (data_rx.u == 2 && last_data_u != 2)) //
			expect_pos = SHOOT_PLUCK_MOTOR1.real_position + delta_pos;
		last_data_u = data_rx.u; 
	}
	
	//弹舱盖
	if(infantry.ShootMode == shoot_disabled)
	{
		if(Remote.key.c) //press C to open 
			TIM1->CCR1 = 1790;
		else
			TIM1->CCR1 = 840;
	}
}

/**
	* @brief 17mm 摩擦轮射击初速调节
	* @param None
	* @retval None
	*/
void ShootCtrl_SpeedLimit_17mm(void)
{
    uint16_t SpeedLimit = robot_state.shooter_id1_17mm_speed_limit;
    switch (SpeedLimit)
    {
    case 15:
        infantry.shoot.ShootSpeed = 6000;
        infantry.shoot.ShootSpeedLeft = 6000;
        infantry.shoot.ShootSpeedRight = 6000;
        break;
    case 18:
        infantry.shoot.ShootSpeed = 7000;
        infantry.shoot.ShootSpeedLeft = 7000;
        infantry.shoot.ShootSpeedRight = 7000;
        break;
    case 30:
        infantry.shoot.ShootSpeed = 11500;
        infantry.shoot.ShootSpeedLeft = 12000;
        infantry.shoot.ShootSpeedRight = 11500;
        break;
    default:
        infantry.shoot.ShootSpeed = 6000;
        infantry.shoot.ShootSpeedLeft = 6000;
        infantry.shoot.ShootSpeedRight = 6000;
        break;
    }
    //if(Remote.mouse.press_r == 1) infantry.shoot.ShootSpeed -= 65;
	//infantry.shoot.ShootSpeed = 0;
}

/**
	* @brief 17mm 摩擦轮射频调节
	* @param None
	* @retval None
	*/
//void ShootCtrl_ShootFre_17mm(void)
//{
//	uint16_t SpeedLimit = robot_state.shooter_id1_17mm_speed_limit;
//	switch(SpeedLimit)
//	{
//	case 15: infantry.shoot.FireRate = 2400;break;
//	case 18: infantry.shoot.FireRate = 2400;break;
//	case 30: infantry.shoot.FireRate = 3000;break;
//	default: infantry.shoot.FireRate = 2400;
//	}
//}
