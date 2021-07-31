#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include "can.h"

struct CAN_Motor
{
	short fdbPosition;           //电机的编码器反馈值
	short last_fdbPosition;      //电机上次的编码器反馈值
	short fdbSpeed;              //电机反馈的转速/rpm
	short electric_current;      //电机实际转矩电流
	uint8_t temperature;         //电机温度
	short round;                 //电机转过的圈数
	
	int32_t last_real_position;  //上次真实转过的角度
	int32_t real_position;       //过零处理后的电机转子位置
	struct PID_t speed_pid;      //速度环
	struct PID_t position_pid;   //位置环
	
//	enum CAN_Motor_place_e place;//电机位置
	float line_speed;            //线速度（m/s，根据角速度算出）
	float position_buf[24];        //计算角速度的数组
	int index;                   //数组编号标志
	float velocity;                //用电机编码器计算出来的角速度（单位：度每秒）
};

void MotorEncoderProcess(struct CAN_Motor *motor, uint8_t *RecvData);

extern void MotorParamInit(struct CAN_Motor *motor, \
				float speedP,float speedI,float speedD,float speedErrormax,float speedOutmax, \
				float positionP,float positionI,float positionD,float positionErrormax,float positionOutmax);
extern void MotorParamInitFW(struct CAN_Motor *motor, \
				float speedPrough,float speedPfine, float speedI,float speedD,float speedRangeRough, float speedRangeFine, float speedCompensation, float speedErrormax, float speedErrPreload, float speedOutmax, \
				float positionPrough, float positionPFine, float positionI,float positionD,float positionRangeRough, float positionRangeFine, float positionCompensation, float positionErrormax,float positionErrPreload, float positionOutmax);
extern void CanTransmit_1234(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
extern void CanTransmit_5678(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
extern void CanTransmit_9_12(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);

#endif
