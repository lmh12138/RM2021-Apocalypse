/**
  ******************************************************************************
  * 文件名          : gimbal_interface.c
  * 文件描述        : 步兵机器人云台交互控制
  * 创建时间        : 2020.7.31
  * 作者            : 邓紫龙
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.6.30
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

#include "gimbal.h"
#include "robot.h"

#include "arm_math.h"
#include "kalman_filter.h"

#include "ramp.h"

#include "FreeRTOS.h"
#include "task.h"

#include "kalman.h"

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)

//
extern struct Robot_t infantry;

extern KALMAN_t Kalman_vision_distance;
extern KALMAN_t Kalman_data_x_buff;

const float r2d=180.0f/3.14159f;
const float d2r=3.14159f/180.0f;
const float G_sz=9.7887f;

float Last_data_y = 0.0;
float Last_data_x = 0.0;
float gimbal_kalman_yaw_buff;

float debug_dropoffset = 0.0f;
float debug_CamGun = 0.0f;
float debug_pitch = 0.0f;
float debug_origin_pitch_angle = 0.0f;
/***************自瞄******************/
//debug用
float Kal_yaw = 0.0;
float Kal_yaw_angle = 0.0; 
float Kal_yaw_speed = 0.0;

float Cloud_Angle_Target; //预测yaw控制目标 

float DELTA_K_15 = 0.002f;	//下坠补偿相关参数
float DELTA_K_18 = 0.0017f;
float DELTA_K_30 = 0.00012;

float DELTA_B_15 = 3.0f;
float DELTA_B_18 = 2.8f;
float DELTA_B_30 = 0.60f;

float DELTA_Angle = 0.000f;//云台角度补偿系数
float DELTA_Angle_Sentry = 1.5f;//打哨兵云台角度补偿倍数

//误差
float Auto_Error_Yaw[2];//now/last
float Auto_Distance;//目标距离

//自瞄突然开启,卡尔曼滤波开启延时
uint16_t Auto_KF_Delay = 0;

//方便调试
float debug_y_sk;// = 38;//35;//30;//移动预测系数,越大预测越多
float debug_y_sb_sk;//哨兵预测系数
float debug_y_sb_brig_sk;//桥头哨兵
float debug_auto_err_y;// = 10;//15;//10;//15;//yaw角度过大关闭预测
float debug_kf_delay;// = 150;//100;//200;//120;//150;//预测延时开启
float debug_kf_speed_yl;//yaw速度过低关闭预测
float debug_kf_speed_yl_sb;//抬头打哨兵时减小最低可开预测量
float debug_kf_speed_yh;//yaw速度过高关闭预测
float debug_kf_y_angcon;// = 130;//125;//115;//135;//yaw预测量限幅

float speed_para_yaw; //弹速参数 根据不同初速调节

float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//卡尔曼滤波速度测量值
float *yaw_kf_result, *pitch_kf_result;//二阶卡尔曼滤波结果,0角度 1速度


/*************卡尔曼滤波**************/
/*二阶卡尔曼*/
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2

speed_calc_data_t Vision_Yaw_speed_Struct;


kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}//500 1000
};//初始化yaw的部分kalman参数

kalman_filter_t yaw_kalman_filter;

/*自动打弹用的一些标志位*/
uint8_t Mobility_Prediction_Yaw = 0;//预测是否开启标志位
uint8_t Mobi_Pre_Yaw_Fire = 0;//默认预测没到位，禁止开枪

uint16_t mobpre_yaw_left_delay = 0;//向左预测延时判断可开火消抖
uint16_t mobpre_yaw_right_delay = 0;//向右预测延时判断可开火消抖
uint16_t mobpre_yaw_stop_delay = 0;//预测关闭延时判断可开火消抖

/********************************************************************************/

void GIMBAL_InitArgument(void)
{
	debug_y_sk = 60;//45;//35;//14.8;//移动预测系数,越大预测越多 
	debug_y_sb_sk = 70;//55;//58																 
	debug_y_sb_brig_sk = 70;//
	debug_auto_err_y = 110;//角度过大关闭预测
	debug_kf_delay = 100;//预测延时开启
	debug_kf_speed_yl = 0.5;//0.35;//速度过低关闭预测
	debug_kf_speed_yl_sb = 0.3f;//0.2;//
	debug_kf_speed_yh = 5;//速度过高关闭预测
	debug_kf_y_angcon = 220;//125;//115;//135;//预测量限幅
	
//	debug_y_sk = 48;//45;//35;//14.8;//移动预测系数,越大预测越多 
//	debug_y_sb_sk = 72;//55;//58																 
//	debug_y_sb_brig_sk = 90;//
//	debug_auto_err_y = 190;//角度过大关闭预测
//	debug_kf_delay = 90;//预测延时开启
//	debug_kf_speed_yl = 0.55f;//0.35;//速度过低关闭预测
//	debug_kf_speed_yl_sb = 0.3f;//0.2;//
//	debug_kf_speed_yh = 6;//速度过高关闭预测
//	debug_kf_y_angcon = 200;//125;//115;//135;//预测量限幅

  /*自瞄卡尔曼滤波,二阶*/
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
	
	KalmanInit_dis(&Kalman_vision_distance);
	KalmanInit_buff(&Kalman_data_x_buff);
}

float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//计算速度

		S->processed_speed = S->speed;
		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//时间过长则认为速度不变
	}
	//debug_speed = S->processed_speed;
	return S->processed_speed;//计算出的速度
}

/**
  * @brief  是否在自瞄哨兵
  * @param  void
  * @retval TRUE   FALSE
  * @attention 自瞄抬头角度太大则认为在打哨兵
  */
 int GIMBAL_AUTO_PITCH_Sentry(void)
{
	if(GIMBAL_PITCH_MOTOR.fdbPosition - infantry.gimbal.PitchBiasAngle >= 400/*300*/ )//抬头接近限位
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**********************************************************************************/
/**
  * @brief  自瞄控制函数
  * @param  void
  * @retval void
  * @attention 中间坐标(0,0),左正右负,上负下正
  *            只有当新数据来了才在当前实时角度上累加误差当成目标角度
  *            新数据一来,目标角度就实时更新
  */
float debug_y_dk = 450;//yaw距离预测比例，越大预测越少
uint32_t Vision_Time[2];// NOW/LAST

int vision_time_js;
float error_yaw_k   = 1.2f;//7.5;//5.6;//2.2;//误差放大
float debug_kf_y_angle;//yaw预测暂存
float debug_kf_p_angle;//pitch预测暂存

//根据距离调节预测比例和限幅
float yaw_speed_k = 0;
float kf_yaw_angcon = 0;

float debug_kf_angle_temp;//预测角度斜坡暂存量
float debug_kf_angle_ramp = 5;//预测角度斜坡变化量

float kf_speed_yl = 0;//

void GIMBAL_AUTO_Mode_Ctrl(void)
{	
	uint16_t SpeedLimit = robot_state.shooter_id1_17mm_speed_limit;
	if(SpeedLimit != 15 && SpeedLimit != 18 && SpeedLimit != 30)SpeedLimit = 15;

	static float yaw_angle_raw;//卡尔曼滤波角度测量值
	static float yaw_angle_ref;//记录目标角度
	
	float kf_delay_open = 0;
	
	kf_speed_yl = debug_kf_speed_yl;

	//获取角度偏差量,欧拉角类型,过分依赖于视觉的精准度
	Auto_Error_Yaw[NOW] = -(float)(data_rx.x * infantry.gimbal.aim_yaw_sens);
	
	//Auto_Distance = data_rx.z/1000;  
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓数据更新↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	if(Vision_If_Update() == 1)//视觉数据更新了
	{
		//更新目标角度//记录当前时刻的目标位置,为卡尔曼做准备
		yaw_angle_ref   = bmi088.angle.encoder_yaw   + Auto_Error_Yaw[NOW]   * error_yaw_k;
		Vision_Clean_Update_Flag();//一定要记得清零,否则会一直执行
		Vision_Time[NOW] = xTaskGetTickCount();//获取新数据到来时的时间
	}
	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑数据更新↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
	
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓二阶卡尔曼计算↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	if(Vision_Time[NOW] != Vision_Time[LAST])//更新新数据到来的时间
	{
		vision_time_js = Vision_Time[NOW] - Vision_Time[LAST];//计算视觉延迟
		yaw_angle_raw  = yaw_angle_ref;//更新二阶卡尔曼滤波测量值
		Vision_Time[LAST] = Vision_Time[NOW];
	}
	
	//目标速度解算
	if(data_rx.u != 0)//识别到了目标
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, Vision_Angle_Speed_Yaw);
		Kal_yaw_angle = yaw_kf_result[0];
		Kal_yaw_speed = yaw_kf_result[1];
	}
	else
	{
		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), bmi088.angle.encoder_yaw);
		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, bmi088.angle.encoder_yaw, 0);
		Kal_yaw_angle = yaw_kf_result[0];
		Kal_yaw_speed = yaw_kf_result[1];
		debug_kf_angle_temp = 0;
	}
		kf_delay_open = debug_kf_delay;
	
		Auto_KF_Delay++;//滤波延时开启
		
		switch(SpeedLimit)//不同初速预测量相应调整
		{	
			case 15:
				speed_para_yaw = 1.0f;
			break;
			case 18:
				speed_para_yaw = 0.85f;
			break;
			case 30:
				speed_para_yaw = 0.6f;
			break;
			default:
				speed_para_yaw = 1.0f;
			break;
		}

		if(GIMBAL_AUTO_PITCH_Sentry() == 1)
		{
			yaw_speed_k = debug_y_sb_sk;			
			kf_yaw_angcon = debug_kf_y_angcon;
			kf_speed_yl = debug_kf_speed_yl_sb;
		}
		else if(Auto_Distance < 0.4f)
		{
			yaw_speed_k = debug_y_sk * 0.6f;
			kf_yaw_angcon = debug_kf_y_angcon;
			kf_speed_yl = debug_kf_speed_yl * 2;
		}	
		else
		{
			yaw_speed_k = debug_y_sk;
			kf_yaw_angcon = debug_kf_y_angcon;
			kf_speed_yl = debug_kf_speed_yl;
		}	
		/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑二阶卡尔曼计算↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
		
		/*预测开启条件*/
		if(fabs(Auto_Error_Yaw[NOW]) < debug_auto_err_y//debug看 
				&& Auto_KF_Delay > kf_delay_open 
					&& fabs(yaw_kf_result[KF_SPEED]) >= kf_speed_yl 
						&& fabs(yaw_kf_result[KF_SPEED]) < debug_kf_speed_yh )
		{
			if(yaw_kf_result[KF_SPEED]>=0)
			{
				debug_kf_angle_temp = speed_para_yaw * yaw_speed_k * (yaw_kf_result[KF_SPEED] - kf_speed_yl) * 1;//debug_kf_dist;
			}
			else if(yaw_kf_result[KF_SPEED]<0)
			{
				debug_kf_angle_temp = speed_para_yaw * yaw_speed_k * (yaw_kf_result[KF_SPEED] + kf_speed_yl) * 1;//debug_kf_dist;			
			}

			debug_kf_angle_temp = constrain_float(debug_kf_angle_temp, -debug_kf_y_angcon, debug_kf_y_angcon);//预测暂存量限幅
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp, debug_kf_y_angle, debug_kf_angle_ramp);//预测量缓慢变化
			
			debug_kf_y_angle = constrain_float(debug_kf_y_angle, -debug_kf_y_angcon, debug_kf_y_angcon);
			Cloud_Angle_Target = yaw_kf_result[KF_ANGLE] + debug_kf_y_angle;//

			/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓预测到位判断↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
			if( (yaw_kf_result[KF_SPEED]>0) //目标向左移且误差值显示说目标在右边，则说明预测到位置，可打弹
					&& (Auto_Error_Yaw[NOW] < 30.0f) )
			{
				mobpre_yaw_right_delay = 0;//向右预测开火延时重置

				mobpre_yaw_left_delay++;
				if(mobpre_yaw_left_delay > 0/*75*/)//稳定150ms
				{
					Mobi_Pre_Yaw_Fire = 1;//预测到位，可开火
				}
				else
				{
					Mobi_Pre_Yaw_Fire = 0;//预测没到位，不可开火
				}
			}
			else if( (yaw_kf_result[KF_SPEED]<0) //目标向右移且误差值显示说目标在左边，则说明预测到位置，可打弹
						&& (Auto_Error_Yaw[NOW] > -30.0f) )
			{
				mobpre_yaw_left_delay = 0;//向左预测开火延时重置
				
				mobpre_yaw_right_delay++;
				if(mobpre_yaw_right_delay > 0/*75*/)//稳定150ms
				{
					Mobi_Pre_Yaw_Fire = 1;//预测到位，可开火
				}
				else
				{
					Mobi_Pre_Yaw_Fire = 0;//预测没到位，不可开火
				}
			}
			else
			{
				Mobi_Pre_Yaw_Fire = 0;//标记预测没到位，禁止开火
				
				mobpre_yaw_left_delay = 0;//向左预测开火延时重置
				mobpre_yaw_right_delay = 0;//向右预测开火延时重置
			}
			/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑预测到位判断↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
			Mobility_Prediction_Yaw = 1;//标记预测已开启

			mobpre_yaw_stop_delay = 0;//重置静止时的开火延迟
		}
		else
		{
			Cloud_Angle_Target = yaw_angle_ref;		
			Mobility_Prediction_Yaw = 0;//标记预测没开启
			mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
			mobpre_yaw_right_delay = 0;//重置右预测的开火延迟

			if( fabs(Auto_Error_Yaw[NOW]) < 1.5f )//接近目标
			{
				mobpre_yaw_stop_delay++;
				if(mobpre_yaw_stop_delay > 25)//停止稳定50ms
				{
					Mobi_Pre_Yaw_Fire = 1;//此时根据视觉开火标志位做射击判断，记得一定要置TRUE
				}
			}
			else
			{
				Mobi_Pre_Yaw_Fire = 0;//标记没回复到位，禁止开火
			}
		}
	}


float dropOffset()  //初速相关下坠补偿
{
	float angle_dropfix; 
	//float bullet_speed;
	float CamGunComp;
	//float comp_speed = 1.0f; 
	
	
	//Auto_Distance = Kalman_Filter(data_rx.z*0.001f,0,&Kalman_vision_distance);
	Auto_Distance = Kalman_Filter(data_rx.z,0,&Kalman_vision_distance);
	if(Auto_Distance != 0.0f )CamGunComp = atanf(0.1f/(Auto_Distance + 0.07f)); 
	else CamGunComp  = 0.0f; 
	
//	debug_CamGun = CamGunComp  * r2d;

	float origin_pitch_angle = (GIMBAL_PITCH_MOTOR.fdbPosition - infantry.gimbal.PitchBiasAngle) / infantry.gimbal.aim_pitch_sens; //deg
//	debug_origin_pitch_angle = origin_pitch_angle;
//	float fix_pitch_angle = 0.0f;//deg //data_rx.y
//	// ideal pitch angle !!ATTENTION : this angle is equal to  current p_angle + fix_angle return by visual system
//	float pitch_angle = (origin_pitch_angle + fix_pitch_angle) * d2r +  CamGunComp;
//	debug_pitch = pitch_angle; 
//	// use last bullet speed as current bullet speed for calculate
		//float bullet_last_speed  = shoot_data_t.bullet_speed;
		uint16_t bullet_limit_speed = robot_state.shooter_id1_17mm_speed_limit;
////	if(fabs((float)bullet_limit_speed-bullet_last_speed) < 3.0f)
////	{
//		switch(bullet_limit_speed)
//		{
//		case 15: 
//			bullet_speed = 10.5f;
//			comp_speed = 1.0f;
//		break;
//		case 18: 
//			bullet_speed = 13.5f;
//			comp_speed = 0.95f;
//		break;
//		case 30: 
//			bullet_speed = 30.0f;
//			comp_speed = 0.41f;
//		break;
//		default: 
//			bullet_speed = 9.5f;
//			comp_speed = 1.0f;
//		break;
//		}
////}
////	else
////		bullet_speed = (float)bullet_limit_speed-7.5f;

//	//bullet_speed = 30.5f;
//	if(data_rx.z <= 0)return 0.0f;
//	else
//	{	
//		angle_dropfix = asinf( (bullet_speed*bullet_speed*tanf(pitch_angle) + G_sz * (Auto_Distance) * cosf(pitch_angle) ) 
//								/ (bullet_speed*bullet_speed * sqrtf(1+tanf(pitch_angle)*tanf(pitch_angle) ))) ; 
//		
//		angle_dropfix =( (angle_dropfix - pitch_angle)* 0.5f + CamGunComp )*r2d * comp_speed;//30m/s 
//		//angle_dropfix -= pitch_angle ;//* r2d;
//		//angle_dropfix += CamGunComp * r2d * infantry.gimbal.aim_pitch_sens;
//		return angle_dropfix;					
//	}
	
	if(Auto_Distance <= 0)return 0.0f;
	else
	{
		if(GIMBAL_AUTO_PITCH_Sentry() == 1) DELTA_Angle *= DELTA_Angle_Sentry;
		switch(bullet_limit_speed )
		{	
			case 15:
				angle_dropfix = Auto_Distance*DELTA_K_15 + DELTA_B_15 + CamGunComp* r2d;//+ DELTA_Angle * origin_pitch_angle  * 1.f 
			break;
			case 18:
				angle_dropfix = Auto_Distance*DELTA_K_18 + DELTA_B_18  + CamGunComp* r2d;//+ DELTA_Angle * origin_pitch_angle  * 0.7f
			break;
			case 30:
				angle_dropfix = (Auto_Distance*DELTA_K_30 + DELTA_B_30 + CamGunComp* r2d) * 0.6f;//+ DELTA_Angle * origin_pitch_angle  * 0.25f 
			break;
			default:
				angle_dropfix = Auto_Distance*DELTA_K_15 + DELTA_B_15  + CamGunComp* r2d;//+ DELTA_Angle * origin_pitch_angle  * 1.f
			break;
		}
		return angle_dropfix;
	}
	
}


/**
	* @brief 云台控制-Remote
	* @param None
	* @retval None
	*/
void GimbalCtrl_Remote(void)
{
//PITCH
	//bias 记得根据俯仰角改参数以使得俯仰效果一致
	if(Remote.rc.ch3 > CHx_BIAS)
		infantry.gimbal.PitchAngle = infantry.gimbal.PitchBiasAngle + 1.52f*(float)(Remote.rc.ch3 - CHx_BIAS);
	else
		infantry.gimbal.PitchAngle = infantry.gimbal.PitchBiasAngle + 1.0f*(float)(Remote.rc.ch3 - CHx_BIAS);
	//lock
//	infantry.gimbal.PitchAngle += 0.05f*(float)(Remote.rc.ch3 - CHx_BIAS);
	
//YAW
	//全跟随编码器
	/*static float RotateAngle;
	if(infantry.ChassisMode == Run)
	{
		infantry.gimbal.YawAngle = infantry.gimbal.YawBiasAngle - 2.0f*(float)(Remote.rc.ch2 - CHx_BIAS);
		RotateAngle = infantry.gimbal.YawAngle;
	}
	else if(infantry.ChassisMode == RotateRun)
	{
		RotateAngle += (0.00134f*infantry.velocity.rot_speed - 0.01f*(float)(Remote.rc.ch2 - CHx_BIAS));
		while(RotateAngle > 8191)
			RotateAngle -= 8192;
		infantry.gimbal.YawAngle = RotateAngle;
	}*/
	//
	if(infantry.GimbalMode == Follow_Encoder_Mode)
	{
		infantry.gimbal.YawAngle = infantry.gimbal.YawBiasAngle - 2.0f*(float)(Remote.rc.ch2 - CHx_BIAS);
	}
	else if(infantry.GimbalMode == Follow_Gyro_Mode)
		infantry.gimbal.YawAngle -= 0.03f*(float)(Remote.rc.ch2 - CHx_BIAS);
	else if(infantry.GimbalMode == Follow_Chassis)
		infantry.gimbal.YawAngle = infantry.gimbal.YawBiasAngle;
}

/**
	* @brief 云台控制-MouseKey
	* @param None
	* @retval None
	* @todo 
	*/
void GimbalCtrl_MouseKey(void)
{	
	//static int16_t mouse_info_x = 0; //鼠标x暂存量
	if(infantry.GameMode == AutoAim)
	{	
		/*
		int delta_yaw, delta_pitch;
		delta_yaw = GIMBAL_YAW_MOTOR.fdbPosition - infantry.gimbal.YawBiasAngle;
		delta_pitch = GIMBAL_PITCH_MOTOR.fdbPosition - infantry.gimbal.PitchBiasAngle;
		if((delta_pitch<50 || delta_pitch>-50) && (delta_yaw<100 || delta_yaw>-100))
			auto_aim_ready = 1;
		
		if(auto_aim_ready == 0) 
		{
			infantry.gimbal.PitchAngle = infantry.gimbal.PitchBiasAngle;
			infantry.gimbal.YawAngle = infantry.gimbal.YawBiasAngle;
		}*/
		//电脑瞄准状态且已找到目标
		
		

		MINMAX(Remote.mouse.x, -150, 150); 
		debug_dropoffset = dropOffset();
		infantry.gimbal.PitchAngle = (float)GIMBAL_PITCH_MOTOR.fdbPosition + (0.6f*data_rx.y + 0.4f*Last_data_y + dropOffset()) * infantry.gimbal.aim_pitch_sens + (float)Remote.mouse.y;//
		
		//infantry.gimbal.PitchAngle = -bmi088.angle.encoder_pitch + (float)(data_rx.y * infantry.gimbal.aim_pitch_sens*0.95f);
		Last_data_y = data_rx.y; 

		if(infantry.GimbalMode == Follow_Gyro_Mode || infantry.GimbalMode == Gyro2Ecd)
		{	
			GIMBAL_AUTO_Mode_Ctrl();
			infantry.gimbal.YawAngle = Cloud_Angle_Target - 2.0f*(float)Remote.mouse.x; 				
		}
		else	
		infantry.gimbal.YawAngle = (float)GIMBAL_YAW_MOTOR.fdbPosition - (float)(data_rx.x * infantry.gimbal.aim_yaw_sens);
		//Last_data_x = data_rx.x; 
		//KalmanTest();

	}
	else if(infantry.GameMode == AtkBuffActivate)
	{	
		
		infantry.gimbal.PitchAngle = (float)GIMBAL_PITCH_MOTOR.fdbPosition + 1.2f*infantry.gimbal.aim_pitch_sens*(float)data_rx.y + 0.1f*infantry.gimbal.aim_pitch_sens*Last_data_y;
		if(infantry.GimbalMode == Follow_Gyro_Mode || infantry.GimbalMode == Gyro2Ecd)
		{
			//gimbal_kalman_yaw_buff = Kalman_Filter(bmi088.angle.encoder_yaw - 1.2f*infantry.gimbal.aim_yaw_sens*(float)data_rx.x,0,&Kalman_data_x_buff);
			//infantry.gimbal.YawAngle = gimbal_kalman_yaw_buff; 
			infantry.gimbal.YawAngle = bmi088.angle.encoder_yaw - 1.4f*infantry.gimbal.aim_yaw_sens*(float)data_rx.x;//- 0.35f*infantry.gimbal.aim_yaw_sens * Last_data_x 
		}
		else
			infantry.gimbal.YawAngle = (float)GIMBAL_YAW_MOTOR.fdbPosition - (float)(data_rx.x * infantry.gimbal.aim_yaw_sens);
	}
	else //(Remote.key.z == 0 || pc_data_rx.dist == 0)
	{
		MINMAX(Remote.mouse.x, -90, 90); 
		MINMAX(Remote.mouse.y, -90, 90);
	//PITCH
		Auto_KF_Delay = 0;//清零给下次延迟预测用
		if(Remote.key.c)
			{
				infantry.gimbal.PitchAngle = infantry.gimbal.PitchBiasAngle;	
			}
		infantry.gimbal.PitchAngle -= 0.3f*(float)Remote.mouse.y;
	//YAW
		if(infantry.GimbalMode == Follow_Encoder_Mode)
			infantry.gimbal.YawAngle = infantry.gimbal.YawBiasAngle - 25.0f*(float)Remote.mouse.x;
		else if(infantry.GimbalMode == Follow_Gyro_Mode)
		{
			infantry.gimbal.YawAngle -= 0.3f*(float)Remote.mouse.x;
			if(infantry.ChassisMode == Independent)
				return;
			
			//if(Remote.keyboard & Key_Q)
			//	infantry.gimbal.YawAngle += 10;
			//if(Remote.keyboard & Key_E)
			//	infantry.gimbal.YawAngle -= 10;
		}
		else if(infantry.GimbalMode == Follow_Chassis)
			infantry.gimbal.YawAngle = infantry.gimbal.YawBiasAngle;
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), bmi088.angle.encoder_yaw);
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, bmi088.angle.encoder_yaw, 0);
		debug_kf_angle_temp = 0;
		//KalmanTest();
		
	}
}
