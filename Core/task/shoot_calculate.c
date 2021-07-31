/**
  ******************************************************************************
  * 文件名          : shoot_calculate.c
  * 文件描述        : 步兵机器人发射系统电机闭环参数计算
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
#include "math.h"

//
extern struct Robot_t infantry;
extern int expect_pos, cur_pos;

void FrictionWheelStartupLimit(struct CAN_Motor * motor)
{
    if (fabsf(motor->speed_pid.fdb) < infantry.shoot.ShootSpeed * 0.4f)
    {
        motor->speed_pid.outputMax = 3000;
    }
    else if (fabsf(motor->speed_pid.fdb) > infantry.shoot.ShootSpeed * 0.8f)
    {
        motor->speed_pid.outputMax = 10000;
    }
    else
    {
        motor->speed_pid.outputMax = (fabsf(motor->speed_pid.fdb) - infantry.shoot.ShootSpeed * 0.4f)/(infantry.shoot.ShootSpeed * 0.4f) * 7000 + 3000;
    }
}

void ShootParamChange(void)
{
	if(infantry.ShootMode == shoot_disabled)
	{	
		SHOOT_PLUCK_MOTOR1.speed_pid.output = 0;
        SHOOT_PLUCK_MOTOR2.speed_pid.output = 0;
		SHOOT_FRICTION_MOTOR1.speed_pid.output = 0;
		SHOOT_FRICTION_MOTOR2.speed_pid.output = 0;
        SHOOT_FRICTION_MOTOR3.speed_pid.output = 0;
		SHOOT_FRICTION_MOTOR4.speed_pid.output = 0;
		//SHOOT_FRICTION_MOTOR1.speed_pid.output = 0;
		//SHOOT_FRICTION_MOTOR2.speed_pid.output = 0;
		//ShootParamCalculate();
		//CanTransmit_1234(&hcan1, 0, 0, 0, 0);
		return;
	}
	else if(infantry.ShootMode == shoot_enabled)
	{
		//摩擦轮
		SHOOT_FRICTION_MOTOR1.speed_pid.ref = -infantry.shoot.ShootSpeedRight;//Right Down
		SHOOT_FRICTION_MOTOR2.speed_pid.ref = infantry.shoot.ShootSpeedLeft;//Left Down
        SHOOT_FRICTION_MOTOR3.speed_pid.ref = infantry.shoot.ShootSpeedRight;//Right up
		SHOOT_FRICTION_MOTOR4.speed_pid.ref = -infantry.shoot.ShootSpeedLeft;//Left up

        FrictionWheelStartupLimit(&SHOOT_FRICTION_MOTOR1);
        FrictionWheelStartupLimit(&SHOOT_FRICTION_MOTOR2);
        FrictionWheelStartupLimit(&SHOOT_FRICTION_MOTOR3);
        FrictionWheelStartupLimit(&SHOOT_FRICTION_MOTOR4);

		//拨弹
		if(infantry.WorkState == MouseKeyControl && infantry.ShootWay != ContinuousShoot)
		{
			if(expect_pos > cur_pos)
				SHOOT_PLUCK_MOTOR2.position_pid.ref = expect_pos; 
		    else
				SHOOT_PLUCK_MOTOR2.position_pid.ref = cur_pos;       //消除位置环抖动
		}
		else
        {
            if (infantry.shoot.FireRate == 0 && infantry.shoot.fireRateLeft !=0)
            {
                SHOOT_PLUCK_MOTOR1.speed_pid.ref = -infantry.shoot.fireRateLeft;
            }
            else
            {
                SHOOT_PLUCK_MOTOR1.speed_pid.ref = -infantry.shoot.FireRate;
            }
            if (infantry.shoot.FireRate == 0 && infantry.shoot.fireRateRight !=0)
            {
                SHOOT_PLUCK_MOTOR2.speed_pid.ref = infantry.shoot.fireRateRight;
            }
            else
            {
                SHOOT_PLUCK_MOTOR2.speed_pid.ref = infantry.shoot.FireRate;
            }
        }

		ShootParamCalculate();
		//CanTransmit_1234(&hcan1, SHOOT_FRICTION_MOTOR1.speed_pid.output, SHOOT_FRICTION_MOTOR2.speed_pid.output, SHOOT_FRICTION_MOTOR3.speed_pid.output, SHOOT_FRICTION_MOTOR4.speed_pid.output);
        //CanTransmit_5678(&hcan1, SHOOT_PLUCK_MOTOR1.speed_pid.output, SHOOT_PLUCK_MOTOR2.speed_pid.output,0, 0);
	}
}

/**
	* @brief 云台发射系统电机输出数据计算
	* @param None
	* @retval None
	*/
void ShootParamCalculate(void)
{
	if(infantry.WorkState == MouseKeyControl && infantry.ShootWay != ContinuousShoot)
	{
	    SHOOT_PLUCK_MOTOR2.position_pid.fdb = (float)SHOOT_PLUCK_MOTOR2.real_position;
        PID_Calc(&SHOOT_PLUCK_MOTOR2.position_pid);
        SHOOT_PLUCK_MOTOR2.speed_pid.ref = SHOOT_PLUCK_MOTOR2.position_pid.output;
        SHOOT_PLUCK_MOTOR2.speed_pid.fdb = (float)SHOOT_PLUCK_MOTOR2.fdbSpeed;
	}
	else
    {
		SHOOT_PLUCK_MOTOR1.speed_pid.fdb = SHOOT_PLUCK_MOTOR1.fdbSpeed;
        SHOOT_PLUCK_MOTOR2.speed_pid.fdb = SHOOT_PLUCK_MOTOR2.fdbSpeed;
    }
	
	SHOOT_FRICTION_MOTOR1.speed_pid.fdb = SHOOT_FRICTION_MOTOR1.fdbSpeed;
	SHOOT_FRICTION_MOTOR2.speed_pid.fdb = SHOOT_FRICTION_MOTOR2.fdbSpeed;
    SHOOT_FRICTION_MOTOR3.speed_pid.fdb = SHOOT_FRICTION_MOTOR3.fdbSpeed;
	SHOOT_FRICTION_MOTOR4.speed_pid.fdb = SHOOT_FRICTION_MOTOR4.fdbSpeed;
	PID_Calc(&SHOOT_PLUCK_MOTOR1.speed_pid);
    PID_Calc(&SHOOT_PLUCK_MOTOR2.speed_pid);
    PID_Calc(&SHOOT_FRICTION_MOTOR1.speed_pid);
    PID_Calc(&SHOOT_FRICTION_MOTOR2.speed_pid);
    PID_Calc(&SHOOT_FRICTION_MOTOR3.speed_pid);
    PID_Calc(&SHOOT_FRICTION_MOTOR4.speed_pid);
}
