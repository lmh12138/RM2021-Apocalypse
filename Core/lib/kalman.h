/**
  ******************************************************************************
  * 文件名        ：Kalman.h
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

#ifndef _KALMAN_H_
#define _KALMAN_H_

typedef struct 
{
	float dt;            //卡尔曼滤波器采样时间
	float angle;
	float angle_dot;     //角度及角速度
	float P[2][2];
	float Pdot[4];
	float Q_angle;
	float Q_gyro;        //角度置信度，角速度置信度
	float R_angle;
	float C_0;
	float q_bias;
	float angle_err;
	float PCt_0;
	float PCt_1;
	float E;
	float K_0;
	float K_1;
	float t_0;
	float t_1;
}KALMAN_t;

extern void KalmanInit(KALMAN_t *Kalman);
extern void KalmanInit_dis(KALMAN_t *Kalman);
extern void KalmanInit_buff(KALMAN_t *Kalman);
extern float Kalman_Filter(float angle_m, float gyro_m, KALMAN_t *Kalman);

#endif
