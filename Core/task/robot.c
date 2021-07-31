/**
  ******************************************************************************
  * 文件名          : robot.c
  * 文件描述        : 步兵机器人参数设置
  * 创建时间        : 2020.7.31
  * 作者            : 邓紫龙
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.5.27
  * 修改人          : 邓紫龙
  ******************************************************************************
  * 1.本代码基于STM32F407IGH6TR开发，编译环境为Keil 5，基于FreeRTOS进行开发。
  * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
	* 3.本代码包含大量中文注释，请以UTF-8编码格式打开
	* 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
	* 3号ddog步兵专用
  ******************************************************************************
  */

#include "robot.h"
#include "supervise.h"
#include <string.h>
#include <cmath>

//ctrl msg
struct Robot_t infantry;
//extern struct PID_FW_t leftPID, rightPID;

/**
	* @brief 机器人数据初始化
	* @param none
	* @retval none
	*/
void RobotParamInit(void)
{
	//自适应悬挂车
	infantry.gimbal.PitchBiasAngle = 4931;
	infantry.gimbal.YawBiasAngle = 819;
	infantry.gimbal.PitchAngle_highest = infantry.gimbal.PitchBiasAngle + 1000;
	infantry.gimbal.PitchAngle_lowest = infantry.gimbal.PitchBiasAngle - 500;
	//自瞄参数
	infantry.gimbal.aim_pitch_sens = 22.756f;
	infantry.gimbal.aim_yaw_sens = 22.756f;
	
	memset(&CHASSIS_MOTOR1, 0, sizeof(CHASSIS_MOTOR1));
	memset(&CHASSIS_MOTOR2, 0, sizeof(CHASSIS_MOTOR2));
	memset(&CHASSIS_MOTOR3, 0, sizeof(CHASSIS_MOTOR3));
	memset(&CHASSIS_MOTOR4, 0, sizeof(CHASSIS_MOTOR4));
	memset(&GIMBAL_PITCH_MOTOR, 0, sizeof(GIMBAL_PITCH_MOTOR));
	memset(&GIMBAL_YAW_MOTOR, 0, sizeof(GIMBAL_YAW_MOTOR));
	memset(&SHOOT_PLUCK_MOTOR1, 0, sizeof(SHOOT_PLUCK_MOTOR1));
    memset(&SHOOT_PLUCK_MOTOR2, 0, sizeof(SHOOT_PLUCK_MOTOR2));
	memset(&SHOOT_FRICTION_MOTOR1, 0, sizeof(SHOOT_FRICTION_MOTOR1));
	memset(&SHOOT_FRICTION_MOTOR2, 0, sizeof(SHOOT_FRICTION_MOTOR2));
    memset(&SHOOT_FRICTION_MOTOR3, 0, sizeof(SHOOT_FRICTION_MOTOR3));
	memset(&SHOOT_FRICTION_MOTOR4, 0, sizeof(SHOOT_FRICTION_MOTOR4));
	
	MotorParamInit(&CHASSIS_MOTOR1,	18,0,0,0,11000,	0,0,0,0,0);
	MotorParamInit(&CHASSIS_MOTOR2,	18,0,0,0,11000, 0,0,0,0,0);
	MotorParamInit(&CHASSIS_MOTOR3,	18,0,0,0,11000, 0,0,0,0,0);
	MotorParamInit(&CHASSIS_MOTOR4,	18,0,0,0,11000, 0,0,0,0,0);

	MotorParamInit(&GIMBAL_PITCH_MOTOR,	240,0.2,0,30000,20000,		0.49,0.05,0.3,300,600);
	MotorParamInit(&GIMBAL_YAW_MOTOR,	230,0.2,3.0,100000,30000, 0.45,0.0018,0.45,15000,1600);
	
    MotorParamInitFW(&SHOOT_PLUCK_MOTOR1,8,2,0.1,0,1500,500,500,131072,0,32767,0,0.5,0,0.1,100000,100000,0,131072,0,3000);
    MotorParamInitFW(&SHOOT_PLUCK_MOTOR2,8,2,0.1,0,1500,500,500,131072,0,32767,0,0.5,0,0.1,100000,100000,0,131072,0,3000);
    MotorParamInitFW(&SHOOT_FRICTION_MOTOR1, 2.5, 2.0, 0.01, 0.01, 1000, 300, 0, 131072, 33000, 10000, 0,0,0,0,0,0,0,0,0,0);
    MotorParamInitFW(&SHOOT_FRICTION_MOTOR2, 2.5, 2.0, 0.01, 0.01, 1000, 300, 0, 131072, 33000, 10000, 0,0,0,0,0,0,0,0,0,0);
    MotorParamInitFW(&SHOOT_FRICTION_MOTOR3, 2.5, 2.0, 0.01, 0.01, 1000, 300, 0, 131072, 33000, 10000, 0,0,0,0,0,0,0,0,0,0);
    MotorParamInitFW(&SHOOT_FRICTION_MOTOR4, 2.5, 2.0, 0.01, 0.01, 1000, 300, 0, 131072, 33000, 10000, 0,0,0,0,0,0,0,0,0,0);
	
	infantry.WorkState = STOP;
	infantry.ChassisMode = Run;
	infantry.GimbalMode = Follow_Gyro_Mode;
	infantry.ShootMode = shoot_disabled;
	infantry.ShootWay = ContinuousShoot;
	infantry.GameMode = Normal;
}

void RobotStateChange(void)
{
	infantry.Last_WorkState = infantry.WorkState;
	if(infantry.Last_WorkState == STOP)
	{
		//使进入工作状态后按当前位置校准底盘
		if(infantry.GimbalMode == Follow_Gyro_Mode)
			infantry.gimbal.YawAngle = bmi088.angle.encoder_yaw;
		else if(infantry.GimbalMode == Follow_Encoder_Mode)
			infantry.gimbal.YawAngle = GIMBAL_YAW_MOTOR.fdbPosition;
		
		//infantry.gimbal.PitchAngle = infantry.gimbal.PitchBiasAngle;
	}
	
	if(bmi088.accBiasFound == 0 || Remote.inputmode == RC_Stop || Is_Error(1<<GYRO_INDEX) || Is_Error(1<<RC_INDEX))
	{
		infantry.WorkState = STOP;
		infantry.ShootMode = shoot_disabled;
		//shoot
		SHOOT_PLUCK_MOTOR1.speed_pid.output = 0;
		SHOOT_FRICTION_MOTOR1.speed_pid.output = 0;
		SHOOT_FRICTION_MOTOR2.speed_pid.output = 0;
		return;
	}
	else if(Remote.inputmode == RC_Remote)
		infantry.WorkState = RemoteControl;
	else if(Remote.inputmode == RC_MouseKey)
	{
		infantry.WorkState = MouseKeyControl;
		static enum Game_Mode_e last_GameMode = Normal;
		if(Remote.key.x == 0 && Remote.key.f == 1 && data_rx.u != 0) //press F switch to AutoAim_Ready Mode 
			infantry.GameMode = AutoAim;
		else if( (Remote.key.g == 1 || Remote.key.b == 1) && (Remote.keyboard&Key_ECtrl) == 0)//g小符 b大符 //&&
			infantry.GameMode = AtkBuffActivate;
		else
			infantry.GameMode = Normal;
			
		if (infantry.GameMode != last_GameMode)
		{
			switch(infantry.GameMode)
			{
			case AutoAim:
				MotorParamInit(&GIMBAL_PITCH_MOTOR,	240,0.2,0,30000,20000,	0.24,0.0,0.14,300,600);
				//MotorParamInit(&GIMBAL_YAW_MOTOR,	140,0.017,6.0,120000,25000, 	0.30,0.0015,0.65,50000,500);
			MotorParamInit(&GIMBAL_YAW_MOTOR,	230,0.2,3.0,100000,30000, 0.3,0.0005,0.60,15000,1600);
			break;
			case AtkBuffActivate:
				data_rx.x = 0; data_rx.y = 0;
				MotorParamInit(&GIMBAL_PITCH_MOTOR,	220,0.5,0,3000,18000,		0.18,0.0,0.15,300,600);
				MotorParamInit(&GIMBAL_YAW_MOTOR,	230,0.2,3.0,100000,30000,	0.36,0.0005,0.32,50000,1600);
			break;
			case Normal:
				//MotorParamInit(&GIMBAL_PITCH_MOTOR,	140,0.05,0,30000,18000,		0.50,0.0,0.3,300,600);
				MotorParamInit(&GIMBAL_PITCH_MOTOR,	240,0.2,0,30000,20000,		0.49,0.05,0.3,300,600);
				//MotorParamInit(&GIMBAL_YAW_MOTOR,	230,0.2,3.0,100000,30000, 0.34,0.0005,0.52,50000,500);
				MotorParamInit(&GIMBAL_YAW_MOTOR,	230,0.2,3.0,100000,30000, 0.36,0.0005,0.45,15000,1600);
			break;
			}
			last_GameMode = infantry.GameMode;
		}
	}
	
	//修改工作状态-remote拨码开关
	if(Remote.rc.s1 == 1)
		infantry.ShootMode = shoot_disabled;
	else
		infantry.ShootMode = shoot_enabled;
	
	if(Remote.rc.s1 == 2 || (Remote.inputmode == RC_MouseKey && Remote.key.r == 1))
		infantry.ChassisMode = RotateRun;
	else
	{
		if(Remote.key.v == 1 && Remote.key.z == 0)
			infantry.ChassisMode = Independent;
		else if(Remote.key.z == 1 && Remote.key.v == 0)
			infantry.ChassisMode = ClimbSpeedAcc;
		else
			infantry.ChassisMode = Run;
	}
	
	//根据底盘模式修改云台跟随模式

//正常运动跟编码器。小陀螺跟陀螺仪
//	static enum Gimbal_Mode_e last_GimbalMode = Follow_Encoder_Mode;
//	if(infantry.ChassisMode == Run)
//	{
//		if(last_GimbalMode == Follow_Gyro_Mode)
//			infantry.GimbalMode = Gyro2Ecd;
//		
//		if(infantry.GimbalMode == Gyro2Ecd)
//		{
//			if(fabs((double)infantry.gimbal.Yaw_offset) < 5)
//			{
//				if(Remote.inputmode == RC_MouseKey && Remote.key.x == 1)
//					infantry.GimbalMode = Follow_Chassis;	//press X switch to Follow_Chassis Mode 
//				else infantry.GimbalMode = Follow_Encoder_Mode;
//			}
//		}
//		else
//		{
//			if(Remote.inputmode == RC_MouseKey && Remote.key.x == 1)
//				infantry.GimbalMode = Follow_Chassis;	//press X switch to Follow_Chassis Mode 
//			else infantry.GimbalMode = Follow_Encoder_Mode;
//		}
//	}
//	else if(infantry.ChassisMode == RotateRun)
//	{
//		infantry.GimbalMode = Follow_Gyro_Mode;
//		if(last_GimbalMode != Follow_Gyro_Mode)
//			infantry.gimbal.YawAngle = bmi088.angle.encoder_yaw;
//	}
//	last_GimbalMode = infantry.GimbalMode;

//全小陀螺模式
	static enum Gimbal_Mode_e last_GimbalMode = Follow_Encoder_Mode;

	if(last_GimbalMode != Follow_Gyro_Mode){
		if(infantry.GimbalMode == Follow_Gyro_Mode)
			infantry.gimbal.YawAngle = bmi088.angle.encoder_yaw;
	}
	if(infantry.ChassisMode == Run && Remote.inputmode == RC_MouseKey && Remote.key.x == 1)//press X switch to Follow_Chassis Mode
		infantry.GimbalMode = Follow_Chassis;	
	else 
		infantry.GimbalMode = Follow_Gyro_Mode;
	last_GimbalMode = infantry.GimbalMode;
}
