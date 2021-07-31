#include "motor.h"
#include "bsp_can.h"
#include "supervise.h"

struct CAN_Motor can1_motor_1;
struct CAN_Motor can1_motor_2;
struct CAN_Motor can1_motor_3;
struct CAN_Motor can1_motor_4;
struct CAN_Motor can1_motor_5;
struct CAN_Motor can1_motor_6;
struct CAN_Motor can1_motor_7;
struct CAN_Motor can1_motor_9;

struct CAN_Motor can2_motor_1;
struct CAN_Motor can2_motor_2;
struct CAN_Motor can2_motor_3;
struct CAN_Motor can2_motor_4;
struct CAN_Motor can2_motor_5;
struct CAN_Motor can2_motor_6;
struct CAN_Motor can2_motor_7;
struct CAN_Motor can2_motor_9;

CAN_TxHeaderTypeDef TxMessage1_4 = {
	.DLC=0x08,
	.StdId=0x200,
	.IDE=CAN_ID_STD,
	.RTR=CAN_RTR_DATA
};

CAN_TxHeaderTypeDef TxMessage5_8 = {
	.DLC=0x08,
	.StdId=0x1FF,
	.IDE=CAN_ID_STD,
	.RTR=CAN_RTR_DATA
};

CAN_TxHeaderTypeDef TxMessage9_12 = {
	.DLC=0x08,
	.StdId=0x2FF,
	.IDE=CAN_ID_STD,
	.RTR=CAN_RTR_DATA
};

/**
  * @brief 电机PID参数初始化
  * @param motor 电机数据结构体
  * @param speedP
  * @retval None
  */
void MotorParamInit(struct CAN_Motor *motor, \
		float speedP,float speedI,float speedD,float speedErrormax,float speedOutmax, \
		float positionP,float positionI,float positionD,float positionErrormax,float positionOutmax)
{
	motor->speed_pid.KP = speedP;
	motor->speed_pid.KI = speedI;
	motor->speed_pid.KD = speedD;
	motor->speed_pid.error_max = speedErrormax;
	motor->speed_pid.outputMax = speedOutmax;
	motor->speed_pid.PID_Mode = PID_POSITION;
	motor->position_pid.KP = positionP;
	motor->position_pid.KI = positionI;
	motor->position_pid.KD = positionD;
	motor->position_pid.error_max = positionErrormax;
	motor->position_pid.outputMax = positionOutmax;
	motor->position_pid.PID_Mode = PID_POSITION;
}
/**
  * @brief 电机PID参数初始化
  * @param motor 电机数据结构体
  * @param speedPrough 速度环P粗调，必须大于Fine
  * @param speedPfine 速度环P细调
  * @param speedI 速度环I
  * @param speedD 速度环D
  * @param speedRangeRough 速度环粗调范围
  * @param speedRangeFine 速度环细调范围
  * @param speedCompensation 速度环补偿
  * @param speedErrormax 速度环积分上限
  * @param speedErrPreload 速度环积分预载值
  * @param speedOutmax 速度环输出上限
  * @param positionPrough 位置环P粗调，必须大于Fine
  * @param positionPfine 位置环P细调
  * @param positionI 位置环I
  * @param positionD 位置环D
  * @param positionRangeRough 位置环粗调范围
  * @param positionRangeFine 位置环细调范围
  * @param positionCompensation 位置环补偿
  * @param positionErrormax 位置环积分上限
  * @param positionErrPreload 位置环积分预载值
  * @param positionOutmax 位置环输出上限
  * @retval None
  */
void MotorParamInitFW(struct CAN_Motor *motor, \
				float speedPrough,float speedPfine, float speedI,float speedD,float speedRangeRough, float speedRangeFine, float speedCompensation, float speedErrormax, float speedErrPreload, float speedOutmax, \
				float positionPrough, float positionPFine, float positionI,float positionD,float positionRangeRough, float positionRangeFine, float positionCompensation, float positionErrormax,float positionErrPreload, float positionOutmax)
{
    motor->speed_pid.KP = speedPrough;
	motor->speed_pid.KI = speedI;
	motor->speed_pid.KD = speedD;
	motor->speed_pid.error_max = speedErrormax;
	motor->speed_pid.outputMax = speedOutmax;
    motor->speed_pid.KP_fine = speedPfine;
    motor->speed_pid.range_rough = speedRangeRough;
    motor->speed_pid.range_fine = speedRangeFine;
    motor->speed_pid.compensation = speedCompensation;
    motor->speed_pid.error_preload = speedErrPreload;
	motor->speed_pid.PID_Mode = PID_COMP_POSITION;
	motor->position_pid.KP = positionPrough;
	motor->position_pid.KI = positionI;
	motor->position_pid.KD = positionD;
	motor->position_pid.error_max = positionErrormax;
	motor->position_pid.outputMax = positionOutmax;
    motor->position_pid.KP_fine = positionPFine;
    motor->position_pid.range_rough = positionRangeRough;
    motor->position_pid.range_fine = positionRangeFine;
    motor->position_pid.compensation = positionCompensation;
    motor->position_pid.error_preload = positionErrPreload;
	motor->position_pid.PID_Mode = PID_COMP_POSITION;
}


/**
	* @brief CAN通信电机的反馈数据具体解析函数
	* @param 电机数据结构体
	* @retval None
	*/
void MotorEncoderProcess(struct CAN_Motor *motor, uint8_t *RecvData)
{
	motor->last_fdbPosition = motor->fdbPosition;
	motor->fdbPosition = RecvData[0]<<8|RecvData[1];
	motor->fdbSpeed = RecvData[2]<<8|RecvData[3];
	motor->electric_current = RecvData[4]<<8|RecvData[5];
	motor->temperature = RecvData[6];

	/* 电机数据过零处理，避免出现位置突变的情况，算出电机已经转过的圈数 */
	if(motor->fdbPosition - motor->last_fdbPosition > 4096)
		motor->round --;
	else if(motor -> fdbPosition - motor->last_fdbPosition < -4096)
		motor->round ++;
	motor->last_real_position = motor->real_position;
	motor->real_position = motor->fdbPosition + motor->round * 8192;	//位置环需要真实角度数据 
	
//	if(motor->place == CHASSIS_Can)
//		motor->line_speed = motor->fdbSpeed*2*3.1415926f*motor_radio/60/19/1000;	//底盘电机线速度计算，得到战车的速度m/s	
//	if(motor->place == GIMBAL_Can)                                     //云台需要编码器数据解算角速度
//	{
	float temp_sum = 0;
	float velocity = 0;                                                //滤波数据保存
	motor->position_buf[motor->index] = motor->real_position - motor->last_real_position;
	motor->index++;
	if(motor->index == 24)
		motor->index = 0;
	
	for(int i=0;i<24;i++)
		temp_sum += motor->position_buf[i];
	velocity = temp_sum * 1.8310546875f;                                        //电机机械角度8192,一圈360,采集24个数据计算，计算公式temp_sum*360*1000/8192/24 = temp_sum*1.83
	motor->velocity = 0.20f*velocity + 0.80f*motor->velocity;            //一阶低通道滤波
//	}
}

/**
	* @brief ID为1~4的电机信号发送函数
	* @param ID为1~4的各个电机的电流数值
	* @retval None
	*/
void CanTransmit_1234(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
	uint8_t TxData[8];
	TxData[0] = (uint8_t)(cm1_iq >> 8);
	TxData[1] = (uint8_t)(cm1_iq & 0xFF);
	TxData[2] = (uint8_t)(cm2_iq >> 8);
	TxData[3] = (uint8_t)(cm2_iq & 0xFF);
	TxData[4] = (uint8_t)(cm3_iq >> 8);
	TxData[5] = (uint8_t)(cm3_iq & 0xFF);
	TxData[6] = (uint8_t)(cm4_iq >> 8);
	TxData[7] = (uint8_t)(cm4_iq & 0xFF);
	uint32_t Can_TxMailbox;
	HAL_CAN_AddTxMessage(hcanx,&TxMessage1_4,TxData, &Can_TxMailbox);
}
/**
	* @brief ID为5~8的电机信号发送函数
	* @param ID为5~8的各个电机的电流数值
	* @retval None
	*/
void CanTransmit_5678(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
	uint8_t TxData[8];
	TxData[0] = (uint8_t)(cm1_iq >> 8);
	TxData[1] = (uint8_t)(cm1_iq & 0xFF);
	TxData[2] = (uint8_t)(cm2_iq >> 8);
	TxData[3] = (uint8_t)(cm2_iq & 0xFF);
	TxData[4] = (uint8_t)(cm3_iq >> 8);
	TxData[5] = (uint8_t)(cm3_iq & 0xFF);
	TxData[6] = (uint8_t)(cm4_iq >> 8);
	TxData[7] = (uint8_t)(cm4_iq & 0xFF);
	uint32_t Can_TxMailbox;
	HAL_CAN_AddTxMessage(hcanx,&TxMessage5_8,TxData, &Can_TxMailbox);
}
/**
	* @brief ID为9~12的电机信号发送函数
	* @param ID为9~12的各个电机的电流数值
	* @retval None
	*/
void CanTransmit_9_12(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
	uint8_t TxData[8];
	TxData[0] = (uint8_t)(cm1_iq >> 8);
	TxData[1] = (uint8_t)(cm1_iq & 0xFF);
	TxData[2] = (uint8_t)(cm2_iq >> 8);
	TxData[3] = (uint8_t)(cm2_iq & 0xFF);
	TxData[4] = (uint8_t)(cm3_iq >> 8);
	TxData[5] = (uint8_t)(cm3_iq & 0xFF);
	TxData[6] = (uint8_t)(cm4_iq >> 8);
	TxData[7] = (uint8_t)(cm4_iq & 0xFF);
	
	uint32_t Can_TxMailbox;
	HAL_CAN_AddTxMessage(hcanx,&TxMessage9_12,TxData, &Can_TxMailbox);
}
