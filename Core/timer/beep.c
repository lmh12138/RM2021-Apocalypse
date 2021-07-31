/**
  ******************************************************************************
  * 文件名          : beep.c
  * 文件描述        : 音乐播放
  * 创建时间        : 2019.8.1
  * 作者            : 邓紫龙
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2019.8.1
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
#include "beep.h"

int music1[8] = {M_Re,M_Mi,M_So,H_Do,0,M_La,H_Do,H_Do};
int music2[14] = {H_Do,0,M_So,0,M_Mi,0,M_Xi,M_Xi,M_Xi,M_Xi,M_La,0,M_Xi,H_Do};
int music3[23] = {L_So,0,M_Do,0,0,M_Mi,M_Re,0,M_Do,0,0,0,M_Mi,0,M_So,0,0,M_La,M_Xi,0,H_Do,H_Do,H_Do};
int music4[63] = {M_Do,0,M_Do,M_Mi,M_Mi,M_Mi,M_Mi,0,
					M_Do,0,M_Do,M_Mi,M_Mi,M_Mi,M_Mi,0,
					M_La,0,M_La,0,M_So,0,M_La,0,
					M_So,M_Do,0,M_Mi,M_Mi,0,0,
					H_Do,0,M_La,M_La,M_So,0,M_La,0,0,
					M_So,0,M_Do,M_Re,M_Re,0,0,
					M_Xi,M_Xi,M_Xi,M_Xi,M_Xi,0,M_Xi,0,M_So,M_Mi,0,M_So,M_So,M_So
					};
int music5[62] = {H_Do,0,M_Xi,0,M_So,0,
					M_So,0,M_La,1145,M_La,0,
					M_So,0,M_So,0,M_La,1145,M_La,0,
					M_So,0,M_So,0,M_La,1145,M_La,0,
					M_La,0,M_La,M_La,M_Xi,M_Xi,H_Do,H_Do,H_Re,H_Re,
					M_Xi,M_Xi,M_Xi,M_Xi,M_So,M_So,M_So,M_So,
	                H_Fa,H_Fa,H_Fa,H_Fa,H_Re,H_Re,H_Re,0,
					H_Re,H_Re,0,H_Re,H_Re,H_Re,H_Mi,H_Mi};//Only My Railgun
int music6[56] = {H_Do,H_Do,H_Do,0,H_Do,0,M_Xi,H_Do,H_Do,0,
					H_Re,0,H_Mi,0,H_Fa,0,
					H_Mi,H_Mi,0,H_Mi,H_Mi,0,M_Xi,0,M_Xi,M_Xi,M_Xi,0,0,
					M_La,M_La,M_La,0,M_La,0,M_So,M_La,M_La,0,
					H_Fa,0,H_Mi,0,H_Re,0,
					H_Re,H_Re,0,H_Do,H_Do,0,H_Re,0,H_Mi,H_Mi,H_Mi
					};
int music_nxt[40] = {M_Do, M_Do,M_Do,M_Do,M_Do,M_Do,M_Do,M_Do,
					M_La,M_La,M_La,M_La,M_La,M_La,M_La,
					M_Fa,M_Fa,M_Fa,M_Fa,M_Fa,
					M_Do,M_Do,M_Do,M_Do,M_Do,M_Do,M_Do,M_Do,
					H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do};

void Beep(void)
{
	static int beep_count = 0;
	if(beep_count < sizeof(MUSIC)/sizeof(int))
	{
		if(MUSIC[beep_count] != 0)
		{
			TIM4->CCR3 = MUSIC[beep_count]/2;
		}
		else
		{
			TIM4->CCR3 = MUSIC[beep_count];
		}
		TIM4->ARR = MUSIC[beep_count];
		beep_count++;
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	}
}
