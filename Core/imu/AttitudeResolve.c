/**
  ******************************************************************************
  * 文件名          : AttitudeResolve.c
  * 文件描述        : 陀螺仪数据解析
  * 创建时间        : 2019.7.31
  * 作者            : 谢胜
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.1.15
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

#include "AttitudeResolve.h"
#include "BMI088driver.h"
#include "imu_heat.h"
#include "kalman.h"
#include <math.h>
#include <string.h>

#define DEG2RAD		0.017453293f
#define RAD2DEG		57.29578f

extern KALMAN_t Kalman_yaw;
extern KALMAN_t Kalman_pitch;
extern KALMAN_t Kalman_roll;

//校准参数
BiasObj	gyroBiasRunning;
float accScale;
//积分参数
float Kp, Ki, exInt, eyInt, ezInt;
//四元数
static float q0, q1, q2, q3;
volatile uint32_t last_update, now_update;

float debug_ex;
float debug_ey;
float debug_ez;
float debug_exInt;
float debug_eyInt;
float debug_ezInt;

//陀螺仪数据结构体
struct IMU_t bmi088;

/**
	* @brief 陀螺仪结构体初始化函数
	* @param 陀螺仪结构体
	* @retval None
	*/
void IMU_init(struct IMU_t *imu)
{
	q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
	Kp = 30.0f;		//比例增益 //20.0f
	Ki = 0.4f;		//积分增益	//0.1f
	exInt = 0.0f;eyInt = 0.0f;ezInt = 0.0f;	//x,y,z积分误差累积
	accScale = 1;
	
	KalmanInit(&Kalman_yaw);         //卡尔曼结构体参数初始化
	KalmanInit(&Kalman_pitch);
	KalmanInit(&Kalman_roll);
	
	memset(&gyroBiasRunning, 0, sizeof(BiasObj));
	memset(imu, 0, sizeof(struct IMU_t));		//陀螺仪数据清空
	
	IMU_Heat_init();
}

/**
	* @brief 计算方差和平均值
	* @param 陀螺仪结构体里面的数据
	* @retval None
	*/
static void sensorsCalculateVarianceAndMean(BiasObj* bias, struct Axis_f* varOut, struct Axis_f* meanOut)
{
	uint32_t i;
	float sum[3] = {0};
	float sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - (sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - (sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - (sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
	* @brief 传感器查找配置值
	* @param None
	* @retval 配置成功返回1，否则返回0
	*/
static int sensorsFindBiasValue(BiasObj* bias)
{
	if (bias->isBufferFilled)
	{
		struct Axis_f mean;
		struct Axis_f variance;
		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			bias->isBiasValueFound = 1;
			return 1;
		}
		else
			bias->isBufferFilled = 0;
	}
	return 0;
}

/**
	* @brief 采集陀螺仪数据样本
	* @param 陀螺仪原始数据
	* @retval 
	*/
int processGyroBias(float gx, float gy, float gz, struct Axis_f *gyroBiasOut)
{
	static int count = 0;
	gyroBiasRunning.buffer[count].x = gx;
	gyroBiasRunning.buffer[count].y = gy;
	gyroBiasRunning.buffer[count].z = gz;
	count++;
	if(count == SENSORS_NBR_OF_BIAS_SAMPLES)
	{
		count = 0;
		gyroBiasRunning.isBufferFilled = 1;
	}
	
//	if(reset == 1)
//	{
//		static int drift_count = 0;
//		drift_count++;
//		if(gx > 150 || gx < -150 || gy > 150 || gy < -150 || gz > 150 || gz < -150)//判断陀螺仪是否在运动
//			drift_count = count = 0;
//		if(drift_count == dirft_time)
//		{
//			angle->last_encoder_yaw = angle->encoder_yaw; //保存陀螺仪yaw的数据
//			angle->yaw_count = 0;                         //对yaw的圈数清零（因为是增量式的）
//			drift_count = 0;                              //重新计时
//			gyroBiasRunning->isBiasValueFound = 0;        //陀螺仪重新计算偏移值
//			gyroBiasRunning->isBufferFilled = 0;
//			count = 0;
//			other->accScaleSum = 0;
//			other->accScale = 1;
//			
//			/* 清空积分误差及四元数误差 */
//			Integral->exInt = Integral->eyInt = Integral->ezInt = 0.0f;
//			Quat->q0 = 1.0f;
//			Quat->q1 = Quat->q2 = Quat->q3 = 0.0f;
//		}
//	}

	if (gyroBiasRunning.isBufferFilled)		//数据采集满了再进行处理
	{
		sensorsFindBiasValue(&gyroBiasRunning);
		gyroBiasOut->x = gyroBiasRunning.bias.x;
		gyroBiasOut->y = gyroBiasRunning.bias.y;
		gyroBiasOut->z = gyroBiasRunning.bias.z;
	}
	return gyroBiasRunning.isBiasValueFound;
}

/**
	* @brief 根据样本计算重力角速度缩放因子
	* @param 加速度计三轴原始数据
	* @retval None
	*/
int processAccScale(float ax, float ay, float az, int *accBiasFound)
{
	static uint16_t accScaleSumCount = 0;
	static float accScaleSum = 0;

	if (*accBiasFound == 0)
	{
		accScaleSum += sqrtf(powf((float)ax, 2) + powf((float)ay, 2) + powf((float)az, 2));
		accScaleSumCount++;
		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			*accBiasFound = 1;
			return 1;
		}
	}
	return 0;
}

/**
	* @brief 陀螺仪和加速度计原始数据解析
	* @param None
	* @retval None
	*/
void imuDataHandle(struct IMU_t* imu)
{
	float acc_raw[3],gyro_raw[3];
	BMI088_read(gyro_raw, acc_raw, &(imu->temp));		//读取IMU数据
	IMU_Heat_Control(imu->temp);	//IMU温控
	if(imu->temp < 36)	//温度未到达正常工作温度则放弃数据
		return;
	
	//获取陀螺仪三轴校准值
	if(!gyroBiasRunning.isBiasValueFound)		//未计算出静态误差矫正值
	{
		processGyroBias(gyro_raw[0], gyro_raw[1], gyro_raw[2], &(imu->gyroBias));
		return;
	}
	//获取加速度计校准值
	if(!imu->accBiasFound)
		processAccScale(acc_raw[0],acc_raw[1],acc_raw[2], &(imu->accBiasFound));
	
	//获取校准后的加速度,将开发板坐标系对应到车体坐标系上
	imu->acc.x = 0.8f*acc_raw[0]/accScale + 0.2f*imu->acc.x;
	imu->acc.y = 0.8f*acc_raw[1]/accScale + 0.2f*imu->acc.y;
	imu->acc.z = 0.8f*acc_raw[2]/accScale + 0.2f*imu->acc.z;
	//获取校准后的角速度,将开发板坐标系对应到车体坐标系上
	imu->gyro.x = 0.8f*(gyro_raw[0]-imu->gyroBias.x) + 0.2f*imu->gyro.x;
	imu->gyro.y = 0.8f*(gyro_raw[1]-imu->gyroBias.y) + 0.2f*imu->gyro.y;
	imu->gyro.z = 0.8f*(gyro_raw[2]-imu->gyroBias.z) + 0.2f*imu->gyro.z;
	
	if(imu->reset == 1)
	{
		static int drift_count = 0;
		
		if(drift_count == 0)	//清空积分误差及四元数误差
		{
			exInt=0.0f; eyInt=0.0f; ezInt=0.0f;
			q0=1.0f; q1=0.0f; q2=0.0f; q3=0.0f;
		}
		
		drift_count++;
		if(fabs(imu->gyro.x) > 0.1f || fabs(imu->gyro.y) > 0.1f || fabs(imu->gyro.z) > 0.1f)//判断陀螺仪是否在运动
			drift_count = 0;
		
		if(drift_count == DRIFT_TIME){
			imu->reset = 0;
			drift_count = 0;
		}
	}
	
	//姿态融合，通过四元数解算姿态角
	imuUpdate(imu->acc, imu->gyro, imu);
}	

/**
	* @brief 根据陀螺仪数据解析出欧拉角函数
	* @param 加速度和角速度结构体
	* @retval None
	*/
void imuUpdate(struct Axis_f acc, struct Axis_f gyro, struct IMU_t *imu)
{
//	imu->gyro.x *= DEG2RAD;
//	imu->gyro.y *= DEG2RAD;
//	imu->gyro.z *= DEG2RAD;
	
	float q0q0 = q0 * q0;
	float q1q1 = q1 * q1;
	float q2q2 = q2 * q2;
	float q3q3 = q3 * q3;

	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q3 = q2 * q3;
	
	float normalise;
	float ex, ey, ez;
	float halfT;
	float vx, vy, vz;
	
	now_update = HAL_GetTick();
	halfT =0.001	;//((float)(now_update - last_update) / 2000.0f)
	last_update = now_update;
	
	//对角速度计数据进行归一化处理
	if(acc.x!=0||acc.y!=0||acc.z!=0)
	{
		normalise = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
		acc.x /= normalise;
		acc.y /= normalise;
		acc.z /= normalise;
	}
	//计算加速度计投影到物体坐标上的各个分量
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	//积误差累计，用以修正陀螺仪数据
	ex = (acc.y*vz - acc.z*vy);
	ey = (acc.z*vx - acc.x*vz);
	ez = (acc.x*vy - acc.y*vx);
	
	debug_ex = ex;
	debug_ey = ey;
	debug_ez = ez;
	//互补滤波 PI
	exInt += ex * Ki * halfT;
	eyInt += ey * Ki * halfT;
	ezInt += ez * Ki * halfT;
	
	debug_exInt = exInt; 
	debug_eyInt = eyInt; 
	debug_ezInt = ezInt; 
	
	gyro.x += Kp*ex + exInt;
	gyro.y += Kp*ey + eyInt;
	gyro.z += Kp*ez + ezInt;
	
	//使用一阶龙格库塔更新四元数
	q0 += (-q1 * gyro.x - q2 * gyro.y - q3 * gyro.z) * halfT;
	q1 += ( q0 * gyro.x + q2 * gyro.z - q3 * gyro.y) * halfT;
	q2 += ( q0 * gyro.y - q1 * gyro.z + q3 * gyro.x) * halfT;
	q3 += ( q0 * gyro.z + q1 * gyro.y - q2 * gyro.x) * halfT;
	//对四元数进行归一化处理
	normalise = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= normalise;
	q1 /= normalise;
	q2 /= normalise;
	q3 /= normalise;
	
	//由四元数求解欧拉角
	imu->attitude.x = -asinf(-2*q1*q3 + 2*q0*q2) * RAD2DEG;	//roll
	imu->attitude.y = atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * RAD2DEG;	//pitch
	imu->attitude.z = atan2f(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * RAD2DEG;	//yaw
	
	
	/* 将欧拉角的范围360扩大到8192 */
	//yaw
	imu->angle.last_yaw = imu->angle.yaw;//yaw轴没必要加卡尔曼
	imu->angle.yaw = imu->attitude.z * 22.7556f;
	if(imu->angle.yaw - imu->angle.last_yaw > 4096)
		imu->angle.yaw_count--;
	else if(imu->angle.yaw - imu->angle.last_yaw < -4096)
		imu->angle.yaw_count++;
	imu->angle.encoder_yaw = imu->angle.yaw + imu->angle.yaw_count * 8192;//imu->angle.last_encoder_yaw + 
	
	//pitch
	imu->angle.last_pitch = imu->angle.pitch;
	imu->angle.pitch = Kalman_Filter(imu->attitude.y,imu->gyro.x,&Kalman_pitch)* 22.7556f;
	//imu->angle.pitch = imu->attitude.y*22.7556f;
	if(imu->angle.pitch - imu->angle.last_pitch > 4096)
		imu->angle.pitch_count--;
	else if(imu->angle.pitch - imu->angle.last_pitch < -4096)
		imu->angle.pitch_count++;
	imu->angle.encoder_pitch = imu->angle.pitch + imu->angle.pitch_count * 8192;
	//roll
	imu->angle.last_roll = imu->angle.roll;
	imu->angle.roll = Kalman_Filter(imu->attitude.x,imu->gyro.y,&Kalman_roll) * 22.7556f;
	if(imu->angle.roll - imu->angle.last_roll > 4096)
		imu->angle.roll_count--;
	else if(imu->angle.roll - imu->angle.last_roll < -4096)
		imu->angle.roll_count++;
	imu->angle.encoder_roll = imu->angle.roll + imu->angle.roll_count * 8192;
	
}
