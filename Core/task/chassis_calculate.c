/**
  ******************************************************************************
  * 文件名          : chassis_calculate.c
  * 文件描述        : 步兵机器人底盘电机闭环参数计算
  * 创建时间        : 2020.7.31
  * 作者            : 邓紫龙
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.7.6
  * 修改人          : 方纬博
  ******************************************************************************
  * 1.本代码基于STM32F407IGH6TR开发，编译环境为Keil 5，基于FreeRTOS进行开发。
  * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
	* 3.本代码包含大量中文注释，请以UTF-8编码格式打开
	* 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
  ******************************************************************************
  */
	
#include "chassis.h"
#include "robot.h"
#include "ramp.h"
#include <math.h>

//
extern struct Robot_t infantry;

/**
	* @brief 底盘数据计算
	* @param none
	* @retval none
	*/
void ChassisParamChange(void)
{	
	if(infantry.WorkState == STOP)
	{
		CHASSIS_MOTOR1.speed_pid.output = 0;
		CHASSIS_MOTOR2.speed_pid.output = 0;
		CHASSIS_MOTOR3.speed_pid.output = 0;
		CHASSIS_MOTOR4.speed_pid.output = 0;
		CanTransmit_1234(&hcan2, 0,0,0,0);
		return;
	}
	
	//底盘运动计算
	float angle = infantry.chassis.turn_angle;
	//static float swing_angle = 1200;
	static enum Chassis_Mode_e last_ChassisMode = Run;
	
	if(infantry.ChassisMode == Run || infantry.ChassisMode == ClimbSpeedAcc)
	{
		infantry.chassis.FBSpeed = infantry.velocity.forward_back_speed;
		infantry.chassis.LRSpeed = infantry.velocity.left_right_speed;
		if(angle > 4096)angle -= 8192;
		else if(angle < -4096)angle += 8192;
		/*
		if(Remote.key.z == 1 && Remote.key.v == 0)	//press v 摆尾 暂时取消 键位挪用
		{
			angle += swing_angle;
			if(fabs(angle) < 140 && swing_angle > 0)
				swing_angle = -1100;
			else if(fabs(angle) < 140 && swing_angle < 0)
				swing_angle = 1100;
		}
		*/
		infantry.chassis.RotateSpeed = -0.0001f* fabsf(angle)* angle* infantry.velocity.turn_speed;
	}
	else if(infantry.ChassisMode == RotateRun)
	{
		if(angle < 0)angle += 8192;	//sin()参数为正,将负的角度值变回正值
		
		angle = angle * 0.00076f;	//0.00076 = 2*pi / 8192,将8192度欧拉角变回弧度角
		infantry.chassis.FBSpeed = ((float)infantry.velocity.forward_back_speed * cos(angle) + (float)infantry.velocity.left_right_speed * sin(angle))*0.62;
		infantry.chassis.LRSpeed = ((float)infantry.velocity.forward_back_speed * -sin(angle) + (float)infantry.velocity.left_right_speed * cos(angle))*0.62;
		//开小陀螺时前后左右减速
		static int rotdir = 1;
		if(last_ChassisMode != RotateRun)
			rotdir *= -1;
		
		//infantry.chassis.RotateSpeed = infantry.velocity.rot_speed * (1 - 0.2f*sin(2*angle)) * rotdir;
		//开小陀螺时旋转减速
		infantry.chassis.RotateSpeed = (infantry.velocity.rot_speed-(fabs(infantry.velocity.forward_back_speed)+fabs(infantry.velocity.left_right_speed))*0.54) * 0.75 * rotdir;
	}
	else if(infantry.ChassisMode == Independent)
	{
		float bia_angle = infantry.gimbal.Yaw_offset * 0.00076f;
		
		infantry.chassis.FBSpeed = (float)infantry.velocity.forward_back_speed * cos(bia_angle) + (float)infantry.velocity.left_right_speed * sin(bia_angle);
		infantry.chassis.LRSpeed = (float)infantry.velocity.forward_back_speed * -sin(bia_angle) + (float)infantry.velocity.left_right_speed * cos(bia_angle);
		infantry.chassis.RotateSpeed = infantry.chassis.RotateSpeed = -0.0001f* fabsf(angle)* angle* infantry.velocity.turn_speed;
	}

	last_ChassisMode = infantry.ChassisMode;
	
if(infantry.GimbalMode == Follow_Chassis){
		MotorParamInit(&CHASSIS_MOTOR1,	25,0,0,0,16384,	0,0,0,0,0);
		MotorParamInit(&CHASSIS_MOTOR2,	25,0,0,0,16384, 0,0,0,0,0);
		MotorParamInit(&CHASSIS_MOTOR3,	25,0,0,0,16384, 0,0,0,0,0);
		MotorParamInit(&CHASSIS_MOTOR4,	25,0,0,0,16384, 0,0,0,0,0);
//		CHASSIS_MOTOR1.speed_pid.ref = infantry.chassis.LRSpeed*1.9f + infantry.chassis.RotateSpeed;
//		CHASSIS_MOTOR3.speed_pid.ref = - infantry.chassis.LRSpeed*1.9f + infantry.chassis.RotateSpeed;
//		CHASSIS_MOTOR2.speed_pid.ref = - infantry.chassis.FBSpeed*1.9f + infantry.chassis.RotateSpeed;
//		CHASSIS_MOTOR4.speed_pid.ref = infantry.chassis.FBSpeed*1.9f + infantry.chassis.RotateSpeed;
		CHASSIS_MOTOR1.speed_pid.ref = infantry.chassis.FBSpeed*0.8 + infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		CHASSIS_MOTOR2.speed_pid.ref = -infantry.chassis.FBSpeed*0.8 + infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		CHASSIS_MOTOR3.speed_pid.ref = -infantry.chassis.FBSpeed*1.5 - infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		CHASSIS_MOTOR4.speed_pid.ref = infantry.chassis.FBSpeed*1.5 - infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		
	}
	else if(infantry.ChassisMode == ClimbSpeedAcc){
		MotorParamInit(&CHASSIS_MOTOR1,	23,0,0,0,13000,	0,0,0,0,0);
		MotorParamInit(&CHASSIS_MOTOR2,	23,0,0,0,13000, 0,0,0,0,0);
		MotorParamInit(&CHASSIS_MOTOR3,	23,0,0,0,13000, 0,0,0,0,0);
		MotorParamInit(&CHASSIS_MOTOR4,	23,0,0,0,13000, 0,0,0,0,0);

		CHASSIS_MOTOR1.speed_pid.ref = infantry.chassis.FBSpeed*0.8 + infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		CHASSIS_MOTOR2.speed_pid.ref = -infantry.chassis.FBSpeed*0.8 + infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		CHASSIS_MOTOR3.speed_pid.ref = -infantry.chassis.FBSpeed*1.2 - infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		CHASSIS_MOTOR4.speed_pid.ref = infantry.chassis.FBSpeed*1.2 - infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		
	}		
	else{
		MotorParamInit(&CHASSIS_MOTOR1,	20,0.5,0,1000,11000, 0,0,0,0,0);
		MotorParamInit(&CHASSIS_MOTOR2,	20,0.5,0,1000,11000, 0,0,0,0,0);
		MotorParamInit(&CHASSIS_MOTOR3,	20,0.5,0,1000,11000, 0,0,0,0,0);
		MotorParamInit(&CHASSIS_MOTOR4,	20,0.5,0,1000,11000, 0,0,0,0,0);

		CHASSIS_MOTOR1.speed_pid.ref = infantry.chassis.FBSpeed + infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		CHASSIS_MOTOR2.speed_pid.ref = -infantry.chassis.FBSpeed + infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		CHASSIS_MOTOR3.speed_pid.ref = -infantry.chassis.FBSpeed - infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		CHASSIS_MOTOR4.speed_pid.ref = infantry.chassis.FBSpeed - infantry.chassis.LRSpeed + infantry.chassis.RotateSpeed;
		ChassisAccelerationLimit();
	}
	//PID calculate & send msg
	ChassisParamCalculate();
	CanTransmit_1234(&hcan2, CHASSIS_MOTOR1.speed_pid.output, CHASSIS_MOTOR2.speed_pid.output,
		CHASSIS_MOTOR3.speed_pid.output, CHASSIS_MOTOR4.speed_pid.output);
}


/**
	* @brief 底盘电机缓启动
	* @param None
	* @retval None
	*/
void ChassisAccelerationLimit(void)
{
	double accMax = 3.0f*(double)robot_state.chassis_power_limit;
	if(accMax < 170.0f)accMax = 170.0f;
	else if(accMax > 270.0f)accMax = 270.0f;	
	if(fabs(CHASSIS_MOTOR1.speed_pid.ref-CHASSIS_MOTOR1.fdbSpeed) > accMax)
		CHASSIS_MOTOR1.speed_pid.ref = CHASSIS_MOTOR1.fdbSpeed + accMax*(CHASSIS_MOTOR1.speed_pid.ref - CHASSIS_MOTOR1.fdbSpeed > 0 ? 1:-1);
	if(fabs(CHASSIS_MOTOR2.fdbSpeed-CHASSIS_MOTOR2.speed_pid.ref) > accMax)
		CHASSIS_MOTOR2.speed_pid.ref = CHASSIS_MOTOR2.fdbSpeed + accMax*(CHASSIS_MOTOR2.speed_pid.ref - CHASSIS_MOTOR2.fdbSpeed > 0 ? 1:-1);
	if(fabs(CHASSIS_MOTOR3.fdbSpeed-CHASSIS_MOTOR3.speed_pid.ref) > accMax)
		CHASSIS_MOTOR3.speed_pid.ref = CHASSIS_MOTOR3.fdbSpeed + accMax*(CHASSIS_MOTOR3.speed_pid.ref - CHASSIS_MOTOR3.fdbSpeed > 0 ? 1:-1);
	if(fabs(CHASSIS_MOTOR4.fdbSpeed-CHASSIS_MOTOR4.speed_pid.ref) > accMax)
		CHASSIS_MOTOR4.speed_pid.ref = CHASSIS_MOTOR4.fdbSpeed + accMax*(CHASSIS_MOTOR4.speed_pid.ref - CHASSIS_MOTOR4.fdbSpeed > 0 ? 1:-1);
}

/**
	* @brief 底盘电机输出数据计算
	* @param None
	* @retval None
	*/
void ChassisParamCalculate(void)
{
	CHASSIS_MOTOR1.speed_pid.fdb = CHASSIS_MOTOR1.fdbSpeed;
	CHASSIS_MOTOR2.speed_pid.fdb = CHASSIS_MOTOR2.fdbSpeed;
	CHASSIS_MOTOR3.speed_pid.fdb = CHASSIS_MOTOR3.fdbSpeed;
	CHASSIS_MOTOR4.speed_pid.fdb = CHASSIS_MOTOR4.fdbSpeed;
	
	PID_Calc(&CHASSIS_MOTOR1.speed_pid);
	PID_Calc(&CHASSIS_MOTOR2.speed_pid);
	PID_Calc(&CHASSIS_MOTOR3.speed_pid);
	PID_Calc(&CHASSIS_MOTOR4.speed_pid);
}
