/**
  ******************************************************************************
  * 文件名          : AttitudeResolve.c
  * 文件描述        : 陀螺仪温度控制
  * 创建时间        : 2021.1.14
  * 作者            : 颜剑燊
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.1.14
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
	
#include <string.h>
#include "imu_heat.h"

struct PID_t imuHeat_pid_t;

void IMU_Heat_init()
{
	memset(&imuHeat_pid_t,0,sizeof(struct PID_t));  //先清零

	imuHeat_pid_t.KP=HEAT_PID_KP;
	imuHeat_pid_t.KI=HEAT_PID_KI;
	imuHeat_pid_t.KD=HEAT_PID_KD;
	imuHeat_pid_t.error_max = 2048;
	imuHeat_pid_t.outputMax=HEAT_PID_MAX_OUT;
	imuHeat_pid_t.error_max=HEAT_PID_MAX_IOUT;
	imuHeat_pid_t.ref = 45.0f;		//设定温度为40
}

void IMU_Heat_Control(float temp)
{
	imuHeat_pid_t.fdb = temp;
	//计算pwm
	PID_Calc(&imuHeat_pid_t);
	TIMx->CCRx = imuHeat_pid_t.output;
}
