/**
  ******************************************************************************
  * 文件名          : chassis_interface.c
  * 文件描述        : 步兵机器人底盘交互控制
  * 创建时间        : 2020.7.31
  * 作者            : 邓紫龙
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.5.9
  * 修改人          : fwb
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
#include <math.h>
#include "super_cap.h"

//
extern struct Robot_t infantry;


#define SHANGJIAO_MECANUM_MOTOR_DISTANCE 0.2724426545f
#define CHASSIS_MOTORSPEED_TO_ROBOT_ANGLESPEED  (1.0f / 1.36f / SHANGJIAO_MECANUM_MOTOR_DISTANCE)
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN  4.14515697348653e-4f 

/**
	* @brief 底盘控制-Remote
	* @param None
	* @retval None
	*/
void ChassisCtrl_Remote(void)
{
	infantry.velocity.forward_back_speed = 16.0f*(float)(Remote.rc.ch1 - CHx_BIAS);
	infantry.velocity.left_right_speed = 16.0f*(float)(Remote.rc.ch0 - CHx_BIAS);
	infantry.velocity.turn_speed = 60;
	infantry.velocity.rot_speed = 5000;
	if(infantry.ChassisMode == RotateRun) //减慢速度
	{
		infantry.velocity.forward_back_speed *= 0.30f;
		infantry.velocity.left_right_speed *= 0.30f;
	}
	infantry.chassis.turn_angle = infantry.gimbal.Yaw_offset;
}

/**
	* @brief 底盘控制-MouseKey
	* @param None
	* @retval None
	*/
void ChassisCtrl_MouseKey(void)
{
	uint16_t max_power = robot_state.chassis_power_limit;
	uint32_t cap_power;
	cap_power = (uint32_t)spuer_cap.vlotage_cap_fdb;
	if(Remote.keyboard & Key_Shift) //key : Shift  按住Speed up
	{	//根据功率 加速幅度区分
		if(infantry.GimbalMode == Follow_Chassis)
			Set_Chassis_param(10000,2400,36,6700);
		else
		{
			switch(max_power)
			{
                case 45: Set_Chassis_param(7600,2400,55,7580);break;
                case 50: Set_Chassis_param(7700,2500,55,7580);break;
                case 55: Set_Chassis_param(7800,2600,55,7920);break;
                case 60: Set_Chassis_param(8000,2750,55,7920);break;
                case 80: Set_Chassis_param(8100,3500,55,8830);break;
                case 100: Set_Chassis_param(8500,3500,55,8830);break;
                case 120: Set_Chassis_param(8800,3500,55,8830);break;
                default: Set_Chassis_param(8000,3500,55,8450);break;
			}	
		}	
	}
	else if(Remote.keyboard & Key_ECtrl){
		//减速模式 用于进站和微调
		Set_Chassis_param(1200,800,30,4000);
	}
	else if(!(Remote.keyboard & Key_Shift || Remote.keyboard & Key_ECtrl))
	{
		switch(max_power)
		{
            case 45: Set_Chassis_param(2450,1350,36,4850);break;
            case 50: Set_Chassis_param(2600,1400,39,5450);break;
            case 55: Set_Chassis_param(2630,1430,43,5730);break;
            case 60: Set_Chassis_param(2650,1450,43,5850);break;			
            case 70: Set_Chassis_param(2760,1550,48,6300);break;
            case 80: Set_Chassis_param(3100,1700,55,6850);break;
            case 100: Set_Chassis_param(3900,2300,55,7000);break;
            case 120: Set_Chassis_param(4800,3200,55,7050);break;
            default: Set_Chassis_param(2550,1350,36,4900);break;
		}
	}
	if(cap_power < 13)Set_Chassis_param(2000,750,20,2250);
	//只按WS时加速 
	if(Remote.keyboard & Key_W && !(Remote.keyboard & Key_A || Remote.keyboard & Key_D))	 //key : W  forward
		infantry.velocity.forward_back_speed *= 1.3f;
	else if(Remote.keyboard & Key_S && !(Remote.keyboard & Key_A || Remote.keyboard & Key_D))	//key : S  backward
		infantry.velocity.forward_back_speed *= -1.3f;
	else if(Remote.keyboard & Key_W)	 //key : W  forward
		infantry.velocity.forward_back_speed *= 1.0f;
	else if(Remote.keyboard & Key_S)	//key : S  backward
		infantry.velocity.forward_back_speed *= -1.0f;
	else
		infantry.velocity.forward_back_speed = 0;

	//只按AD时加速 
	if(Remote.keyboard & Key_D && !(Remote.keyboard & Key_W || Remote.keyboard & Key_S))	//key : D  right
		infantry.velocity.left_right_speed *= -1.5f;
	else if(Remote.keyboard & Key_A && !(Remote.keyboard & Key_W || Remote.keyboard & Key_S))	//key : A  left
		infantry.velocity.left_right_speed *= 1.5f;
	else if(Remote.keyboard & Key_D )	//key : D  right
		infantry.velocity.left_right_speed *= -1.0f;
	else if(Remote.keyboard & Key_A )	//key : A  left
		infantry.velocity.left_right_speed *= 1.0f;
	else
		infantry.velocity.left_right_speed = 0;
	
	if(infantry.GimbalMode == Follow_Chassis)	//云台跟随底盘,仅在MouseKey & Run模式下有效
	{
		//一阶低通滤波
		infantry.chassis.turn_angle = -20.0f*(0.7f*(float)Remote.mouse.x+0.3f*(float)Remote.last_mouse.x);
		if(Remote.keyboard & Key_Q)
			infantry.chassis.turn_angle += 600;
		if(Remote.keyboard & Key_E)
			infantry.chassis.turn_angle -= 600;
	}
	else if(infantry.ChassisMode == Independent)
	{
		if(Remote.keyboard & Key_Q)
			infantry.chassis.turn_angle = 800;
		else if(Remote.keyboard & Key_E)
			infantry.chassis.turn_angle = -800;
		else
			infantry.chassis.turn_angle = 0;
	}
	else
		infantry.chassis.turn_angle = infantry.gimbal.Yaw_offset;
}

void Set_Chassis_param(float fb,float lr,float turn,float rot)
{
	infantry.velocity.forward_back_speed = fb;
	infantry.velocity.left_right_speed = lr;
	infantry.velocity.turn_speed = turn;
	infantry.velocity.rot_speed = rot;
}

/**
 * @brief 里程计
 * @author fwb
 * @note 速度计算功能 注释部分:距离计算功能未完成   
 * @param 
 * @retval 
*/ 
void chassis_distance_calc_task(void)
{
//	int32_t motor_total_ecd[4]= {0,0,0,0};
//	int32_t motor_last_total_ecd[4]= {0,0,0,0};
//	int32_t motor_delta_total_ecd[4]= {0,0,0,0};
//	int32_t d_vx , d_vy , d_wz ;
//  float angle = infantry.chassis.turn_angle;

	float chassis_motor_speed[4] = {0,0,0,0};
	chassis_motor_speed[0] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * CHASSIS_MOTOR1.fdbSpeed;
	chassis_motor_speed[1] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * CHASSIS_MOTOR2.fdbSpeed;
	chassis_motor_speed[2] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * CHASSIS_MOTOR3.fdbSpeed;
	chassis_motor_speed[3] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * CHASSIS_MOTOR4.fdbSpeed;	

//	motor_last_total_ecd[0] = motor_total_ecd[0];
//	motor_total_ecd[0] = CHASSIS_MOTOR1.real_position;
//	motor_delta_total_ecd[0] = motor_total_ecd[0] - motor_last_total_ecd[0];

//	motor_last_total_ecd[1] = motor_total_ecd[1];
//	motor_total_ecd[1] = CHASSIS_MOTOR1.real_position;
//	motor_delta_total_ecd[1] = motor_total_ecd[1] - motor_last_total_ecd[1];

//	motor_last_total_ecd[2] = motor_total_ecd[2];
//	motor_total_ecd[2] = CHASSIS_MOTOR1.real_position;
//	motor_delta_total_ecd[2] = motor_total_ecd[2] - motor_last_total_ecd[2];

//	motor_last_total_ecd[3] = motor_total_ecd[3];
//	motor_total_ecd[3] = CHASSIS_MOTOR1.real_position;
//	motor_delta_total_ecd[3] = motor_total_ecd[3] - motor_last_total_ecd[3];
//			
//	d_vx = +motor_delta_total_ecd[0]-motor_delta_total_ecd[1]-motor_delta_total_ecd[2]+motor_delta_total_ecd[3];
//	d_vy = +motor_delta_total_ecd[0]+motor_delta_total_ecd[1]-motor_delta_total_ecd[2]-motor_delta_total_ecd[3];
//	d_wz = motor_delta_total_ecd[0]+motor_delta_total_ecd[1]+motor_delta_total_ecd[2]+motor_delta_total_ecd[3];
//		
//	infantry.chassis_odom.distance_x += (cos(angle)*d_vx - sin(angle)*d_vy)*0.25f*3.0363e-6f;             //8.0969981169313299869491208890938e-7f ;
//	infantry.chassis_odom.distance_y += (sin(angle)*d_vx + cos(angle)*d_vy)*0.25f*3.0363e-6f;             //8.0969981169313299869491208890938e-7f ;
//	infantry.chassis_odom.distance_wz += d_wz*0.25f*0.8061e-6f;                                   //3.24e-6f;
			  										
	infantry.chassis_odom.Vx = (+ chassis_motor_speed[0] - chassis_motor_speed[1] - chassis_motor_speed[2] + chassis_motor_speed[3]) * 0.25f;
	infantry.chassis_odom.Vy = (+ chassis_motor_speed[0] + chassis_motor_speed[1] - chassis_motor_speed[2] - chassis_motor_speed[3]) * 0.25f;
	infantry.chassis_odom.Wz = (+ chassis_motor_speed[0] + chassis_motor_speed[1] + chassis_motor_speed[2] + chassis_motor_speed[3]) * 0.25f * CHASSIS_MOTORSPEED_TO_ROBOT_ANGLESPEED;//轮子距中心
}


