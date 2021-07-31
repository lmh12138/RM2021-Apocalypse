/**
  ******************************************************************************
  * 文件名        ：Kalman.c
	* 创建时间      ：2019.11.24
	* 作者          ：刘文熠
	*-----------------------------------------------------------------------------
	* 最近修改时间  ：2019.11.24
	* 修改人        ：刘文熠
  ******************************************************************************
  * 1.本代码基于STMF427IIT6开发，编译环境为Keil 5，基于FreeRTOS进行开发。
  * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
	* 3.本代码包含大量中文注释，请以UTF-8编码格式打开
	* 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
	******************************************************************************
  */

#include "Kalman.h"

KALMAN_t Kalman_yaw;
KALMAN_t Kalman_pitch;
KALMAN_t Kalman_roll;

KALMAN_t Kalman_vision_distance;

KALMAN_t Kalman_data_x_buff;

/**
	* @brief 卡尔曼结构体初始化函数
	* @param 卡尔曼结构体
	* @retval None
	*/
void KalmanInit(KALMAN_t *Kalman)
{
//	Kalman->dt = 0.001f;
//	Kalman->Q_angle = 0.001f;
//	Kalman->Q_gyro = 0.005f;
//	Kalman->R_angle = 0.5f;
//	Kalman->C_0 = 1.0f;
	
	Kalman->dt = 0.002f;
	Kalman->Q_angle = 0.01f;
	Kalman->Q_gyro = 0.003f;
	Kalman->R_angle = 0.3f;
	Kalman->C_0 = 1.0f;
}
void KalmanInit_dis(KALMAN_t *Kalman)
{
	Kalman->dt = 0.002f;
	Kalman->Q_angle = 0.01f;
	Kalman->Q_gyro = 0.003f;
	Kalman->R_angle = 0.5f;
	Kalman->C_0 = 1.0f;
}
void KalmanInit_buff(KALMAN_t *Kalman)
{
//	Kalman->dt = 0.001f;
//	Kalman->Q_angle = 0.001f;
//	Kalman->Q_gyro = 0.005f;
//	Kalman->R_angle = 0.5f;
//	Kalman->C_0 = 1.0f;
	
	Kalman->dt = 0.002f;
	Kalman->Q_angle = 0.01f;
	Kalman->Q_gyro = 0.003f;
	Kalman->R_angle = 0.05f;
	Kalman->C_0 = 1.0f;
}

/**
	* @brief 卡尔曼一阶滤波
	* @param 角度angle_x和角速度gyro_y
	* @retval 最优角度
	*/
float Kalman_Filter(float angle_m, float gyro_m, KALMAN_t *Kalman)
{
	Kalman->angle += (gyro_m - Kalman->q_bias) * Kalman->dt;
	
	Kalman->Pdot[0] = Kalman->Q_angle - Kalman->P[0][1] - Kalman->P[1][0];
	Kalman->Pdot[1] = -Kalman->P[1][1];
	Kalman->Pdot[2] = -Kalman->P[1][1];
	Kalman->Pdot[3] = Kalman->Q_gyro;
	Kalman->P[0][0] += Kalman->Pdot[0] * Kalman->dt;
	Kalman->P[0][1] += Kalman->Pdot[1] * Kalman->dt;
	Kalman->P[1][0] += Kalman->Pdot[2] * Kalman->dt;
	Kalman->P[1][1] += Kalman->Pdot[3] * Kalman->dt;
	
	Kalman->angle_err = angle_m - Kalman->angle;
	
	Kalman->PCt_0 = Kalman->C_0 * Kalman->P[0][0];
	Kalman->PCt_1 = Kalman->C_0 * Kalman->P[1][0];
	Kalman->E = Kalman->R_angle + Kalman->C_0 * Kalman->PCt_0;
	Kalman->K_0 = Kalman->PCt_0 / Kalman->E;
	Kalman->K_1 = Kalman->PCt_1 / Kalman->E;
	Kalman->t_0 = Kalman->PCt_0;
	Kalman->t_1 = Kalman->C_0 * Kalman->P[0][1];
	Kalman->P[0][0] -= Kalman->K_0 * Kalman->t_0;
	Kalman->P[0][1] -= Kalman->K_0 * Kalman->t_1;
	Kalman->P[1][0] -= Kalman->K_1 * Kalman->t_0;
	Kalman->P[1][1] -= Kalman->K_1 * Kalman->t_1;
	Kalman->angle += Kalman->K_0 * Kalman->angle_err;    //最优角度
	Kalman->q_bias += Kalman->K_1 * Kalman->angle_err;
	Kalman->angle_dot = gyro_m - Kalman->q_bias;         //最优角速度
	
	return Kalman->angle;
}

