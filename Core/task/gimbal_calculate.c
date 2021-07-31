/**
  ******************************************************************************
  * 文件名          : gimbal_calculate.c
  * 文件描述        : 步兵机器人云台电机闭环参数计算
  * 创建时间        : 2020.7.31
  * 作者            : 邓紫龙
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.5.9
  * 修改人          : 方纬博
  ******************************************************************************
	用于底盘CAN2的ddog
  * 1.本代码基于STM32F407IGH6TR开发，编译环境为Keil 5，基于FreeRTOS进行开发。
  * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
	* 3.本代码包含大量中文注释，请以UTF-8编码格式打开
	* 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
  ******************************************************************************
  */

#include "gimbal.h"
#include "robot.h"
#include "ramp.h"
#include "math.h"

struct ramp_t PitchRamp = RAMP_DEFAULT;     //机器人pitch轴斜坡
struct ramp_t YawRamp = RAMP_DEFAULT;       //机器人yaw轴斜坡

float yaw_speed_in_degps = 0.0f;
int32_t pitchPowerSet = 0;
//
extern struct Robot_t infantry;

/**
	* @brief 底盘电机数据计算
	* @param none
	* @retval none
	*/
void GimbalParamChange(void)
{
    yaw_speed_in_degps = -(bmi088.gyro.z/cosf(atanf(bmi088.gyro.y/bmi088.gyro.z)) * 57.29578f - infantry.chassis_odom.Wz* 57.29578f) * -0.964f;
	if(infantry.WorkState == STOP)
	{
		GIMBAL_YAW_MOTOR.speed_pid.output = 0;
		GIMBAL_PITCH_MOTOR.speed_pid.output = 0;
        pitchPowerSet = 0;
		//CanTransmit_9_12(&hcan1, 0,0,0,0);
		CanTransmit_9_12(&hcan2, 0,0,0,0);
		return;
	}
	
	//Offset_过YAWMotor零点处理(经过电机编码器零点时，反馈值会突变+8192/-8192，消去以使反馈连续)
	infantry.gimbal.Yaw_offset = GIMBAL_YAW_MOTOR.fdbPosition - infantry.gimbal.YawBiasAngle;
	if(GIMBAL_YAW_MOTOR.fdbPosition - GIMBAL_YAW_MOTOR.last_fdbPosition > 4096)
		infantry.gimbal.Yaw_offset -= 8192;
	else if(GIMBAL_YAW_MOTOR.fdbPosition - GIMBAL_YAW_MOTOR.last_fdbPosition < -4096)
		infantry.gimbal.Yaw_offset += 8192;
	
	//Position limited
	if(infantry.gimbal.PitchAngle < infantry.gimbal.PitchAngle_lowest)infantry.gimbal.PitchAngle = infantry.gimbal.PitchAngle_lowest;
	if(infantry.gimbal.PitchAngle > infantry.gimbal.PitchAngle_highest)infantry.gimbal.PitchAngle = infantry.gimbal.PitchAngle_highest;
	
	GIMBAL_PITCH_MOTOR.position_pid.ref = infantry.gimbal.PitchAngle;
	GIMBAL_YAW_MOTOR.position_pid.ref = infantry.gimbal.YawAngle;
	
	GimbalParamCalculate();
    //CanTransmit_9_12(&hcan2, 0, 0, 0, 0);
	CanTransmit_9_12(&hcan2, GIMBAL_YAW_MOTOR.speed_pid.output, 0, 0, 0);
	//CanTransmit_9_12(&hcan2, GIMBAL_PITCH_MOTOR.speed_pid.output, 0, 0, 0);
    pitchPowerSet = 10000 + GIMBAL_PITCH_MOTOR.speed_pid.output ;//
    //CanTransmit_9_12(&hcan1, pitchPowerSet, 0, 0, 0);
}

/**
	* @brief 云台Yaw轴与Pitch轴电机输出数据计算
	* @param None
	* @retval None
	*/
void GimbalParamCalculate(void)
{
	//Pitch
	GIMBAL_PITCH_MOTOR.position_pid.fdb = (float)GIMBAL_PITCH_MOTOR.fdbPosition;
	//GIMBAL_PITCH_MOTOR.position_pid.fdb = -bmi088.angle.encoder_pitch;
	PID_Calc(&GIMBAL_PITCH_MOTOR.position_pid);
	GIMBAL_PITCH_MOTOR.speed_pid.ref = GIMBAL_PITCH_MOTOR.position_pid.output;
	GIMBAL_PITCH_MOTOR.speed_pid.fdb = -bmi088.gyro.x * 57.29578f;//(float)GIMBAL_PITCH_MOTOR.fdbSpeed;
	PID_Calc(&GIMBAL_PITCH_MOTOR.speed_pid);
	//Yaw
	if(infantry.GimbalMode == Follow_Gyro_Mode || infantry.GimbalMode == Gyro2Ecd)
		GIMBAL_YAW_MOTOR.position_pid.fdb = bmi088.angle.encoder_yaw;
	else
		GIMBAL_YAW_MOTOR.position_pid.fdb = (float)GIMBAL_YAW_MOTOR.fdbPosition;
	
	PID_Calc(&GIMBAL_YAW_MOTOR.position_pid);
	GIMBAL_YAW_MOTOR.speed_pid.ref = GIMBAL_YAW_MOTOR.position_pid.output - infantry.chassis_odom.Wz* 57.29578f;
	GIMBAL_YAW_MOTOR.speed_pid.fdb = yaw_speed_in_degps * 0.5f + GIMBAL_YAW_MOTOR.velocity * 0.5f;
	//GIMBAL_YAW_MOTOR.speed_pid.fdb = -bmi088.gyro.z * 57.29578f;	//单位:deg/s
	PID_Calc(&GIMBAL_YAW_MOTOR.speed_pid);
}
