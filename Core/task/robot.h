/**
  ******************************************************************************
  * 文件名          : robot.h
  * 文件描述        : 步兵机器人控制协议
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

#ifndef ROBOT_H
#define ROBOT_H

#include "stm32f4xx.h"
#include "main.h"
#include "bsp_can.h"
#include "motor.h"
#include "remote.h"
#include "AttitudeResolve.h"
#include "pc_communicate.h"
#include "datatypes.h"
#include "referee.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"

//底盘
extern struct CAN_Motor can2_motor_1;
extern struct CAN_Motor can2_motor_2;
extern struct CAN_Motor can2_motor_3;
extern struct CAN_Motor can2_motor_4;
//发射系统
extern struct CAN_Motor can1_motor_1;
extern struct CAN_Motor can1_motor_2;
extern struct CAN_Motor can1_motor_3;
extern struct CAN_Motor can1_motor_4;
extern struct CAN_Motor can1_motor_5;
extern struct CAN_Motor can1_motor_6;
//云台双轴
extern struct CAN_Motor can2_motor_9;
extern struct CAN_Motor can1_motor_7;
//遥控器
extern struct DT7Remote_t Remote;
//陀螺仪
extern struct IMU_t bmi088;
//小电脑
//extern struct PC_Data_TX_t pc_data_tx;
extern SendData data_rx;
//裁判系统
extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
extern ext_game_robot_pos_t game_robot_pos_t;
extern ext_shoot_data_t shoot_data_t;

//电机具体功能宏定义
#define CHASSIS_MOTOR1 can2_motor_1
#define CHASSIS_MOTOR2 can2_motor_2
#define CHASSIS_MOTOR3 can2_motor_3
#define CHASSIS_MOTOR4 can2_motor_4
#define GIMBAL_YAW_MOTOR can2_motor_9
#define GIMBAL_PITCH_MOTOR can1_motor_7
#define SHOOT_PLUCK_MOTOR1 can1_motor_5
#define SHOOT_PLUCK_MOTOR2 can1_motor_6
#define SHOOT_FRICTION_MOTOR1 can1_motor_1
#define SHOOT_FRICTION_MOTOR2 can1_motor_2
#define SHOOT_FRICTION_MOTOR3 can1_motor_3
#define SHOOT_FRICTION_MOTOR4 can1_motor_4

//控制模式
enum WorkState_e
{
	STOP,
	RemoteControl,
	MouseKeyControl
};
enum Chassis_Mode_e
{
	Run,
	RotateRun,
	Independent,
	ClimbSpeedAcc
};
enum Gimbal_Mode_e
{
	//底盘跟随云台
	Follow_Encoder_Mode,
	Follow_Gyro_Mode,
	//云台跟随底盘
	Follow_Chassis,
	//陀螺仪切换到普通
	Gyro2Ecd
};
enum Shoot_Mode_e
{
	shoot_enabled,
	shoot_disabled
};
enum Shoot_Way_e
{
	SingleShoot,
	DoubleShoot,
	TribleShoot,
	ContinuousShoot
};
enum Game_Mode_e
{
	Normal,
	AutoAim,
	AtkBuffActivate
}
;

//各部分运动参数
struct Chassis_t
{
	int FBSpeed;
	int LRSpeed;
	int RotateSpeed;
	int turn_angle;
};
struct Gimbal_t
{
	float PitchAngle;
	float PitchBiasAngle;
	float PitchAngle_lowest;
	float PitchAngle_highest;
	float YawAngle;
	float YawBiasAngle;
	float Yaw_offset;
	float aim_yaw_sens;
	float aim_pitch_sens;
};
struct Shoot_t
{
	float ShootSpeed;
    float ShootSpeedLeft;
    float ShootSpeedRight;
	float FireRate;
    float fireRateLeft;
    float fireRateRight;
    enum shoot_Status_t
    {
        SHOOT_LEFT,
        SHIIT_RIGHT
    }shootStatus;
};

//整体运动速度
struct Robot_Vel
{
	float forward_back_speed;	//前后速度
	float left_right_speed;		//左右速度
	float turn_speed;			//转向速度
	float rot_speed;			//小陀螺速度
};

struct Chassis_odom_info_t
{
  float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vw;
  float distance_x;
  float	distance_y;
  float distance_wz;
  float Vx;
  float Vy;
  float Wz;
};

typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}speed_calc_data_t;


//机器人参数结构体
struct Robot_t
{
	//mode
	enum WorkState_e WorkState;
	enum WorkState_e Last_WorkState;
	enum Chassis_Mode_e ChassisMode;
	enum Gimbal_Mode_e GimbalMode;
	enum Shoot_Mode_e ShootMode;
	enum Shoot_Way_e ShootWay;
	enum Game_Mode_e GameMode;
	//param
	struct Gimbal_t gimbal;
	struct Shoot_t shoot;
	struct Chassis_t chassis;
	//speed - for the whole robot
	struct Robot_Vel velocity;
	struct Chassis_odom_info_t chassis_odom;
};

extern void RobotParamInit(void);
extern void RobotStateChange(void);

#endif
