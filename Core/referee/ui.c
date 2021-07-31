/**
  ******************************************************************************
  * 文件名          : ui.c
  * 创建时间        : 2021.1.23
  * 作者            : 方纬博  
  * ----------------------------------------------------------------------------
  * 最近修改时间    : 2021.5.9
  * 修改人          : 方纬博
  ******************************************************************************
  * 1.本代码基于STM32F407IGH6TR开发，编译环境为Keil 5，基于FreeRTOS进行开发。
  * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
	* 3.本代码包含大量中文注释，请以UTF-8编码格式打开
	* 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
  ******************************************************************************
  * 说明：
  * 
  * 
  ******************************************************************************
  */
#include "ui.h"
#include "robot.h"
#include "super_cap.h"
#include "math.h"

#include <stdarg.h>

extern struct Robot_t infantry;

uint16_t self_ID, self_client_ID;

uint32_t Line3_y_pos = 405;
uint32_t Line4_y_pos = 365;
uint32_t Line5_y_pos = 340;
uint8_t LastLimit = 0;

uint8_t LastChassisState;
uint8_t LastGimbalState;
uint8_t LastShootState;
uint8_t LastAimState;
uint8_t LasrLockAState;
uint8_t LastautoAimState;
uint8_t LastServoState;
uint8_t LastFriState;

char ChassisMsg[30] = {'C','h','a','s','s','i','s',':','r','u','n'};
char GimbalMsg[30] = {'G','i','m','b','a','l',':','G','y','r','o'};
char HatchMsg[30] = {'H','a','t','c','h',':','L','o','c','k'};
char FriMsg[30] = {'F','r','i',':','O','F','F'};

uint32_t single_width = 0;
uint32_t double_width = 0;
uint32_t trible_width = 0;//暂无连发指示

uint32_t ServoMode_switch = 3;
uint32_t AimState_switch = 0;
uint32_t autoAimState_color = 8;

uint32_t FirState_color = 3;// green-ON
uint32_t HatchState_color = 2;// orange-LOCK
uint32_t GimbalState_color = 2;// cyan-Chassis
uint32_t ChassisState_color = 2;// yellow-Rot pink-Cli
uint32_t AtkBuffState_color = 8;// 

uint32_t Cap_power_pos = 0;
//////////////////////////////////////////////////////////////////////////
/*
*						--图形字符配置部分--
*/
//////////////////////////////////////////////////////////////////////////
/**
 * @brief 客户端绘制一个图形配置 超级电容指示线
 * @author fwb
 * @note	   
 * @param	保存一个图形信息的结构体 
 * @retval	none 
*/ 
void One_Pic_set(graphic_data_struct_t* front_sight)
{
	memset(front_sight,0,sizeof(graphic_data_struct_t));
	uint32_t cap_power;
	cap_power = (uint32_t)spuer_cap.vlotage_cap_fdb;
	front_sight->graphic_name[0] = 8;
	front_sight->graphic_name[1] = 2;
	front_sight->graphic_name[2] = 3;
	front_sight->operate_type=1;
	front_sight->graphic_type=0;
	front_sight->color=1;
	front_sight->layer=5;
	front_sight->start_x = (cap_power-16)*33+160;//cap_power+160
	front_sight->start_y = 860;
	front_sight->end_x = (cap_power-16)*33+160;//(uint32_t)cap_power+160
	front_sight->end_y = 900;	
	front_sight->width = 4;	
}

/**
 * @brief 客户端修改一个图形配置 超级电容指示线
 * @author fwb
 * @note	暂未使用 仅使用绘制模式  
 * @param	保存一个图形信息的结构体 
 * @retval	none 
*/ 
void One_Pic_Change(graphic_data_struct_t* front_sight,uint32_t cap_power)
{
	cap_power = (uint32_t)spuer_cap.vlotage_cap_fdb;
	front_sight->graphic_name[0] = 8;
	front_sight->graphic_name[1] = 2;
	front_sight->graphic_name[2] = 3;
	front_sight->operate_type=2;
	front_sight->graphic_type=0;
	front_sight->color=1;
	front_sight->layer=5;
	front_sight->start_x = (cap_power-16)*33+160;//cap_power+
	front_sight->start_y = 860;
	front_sight->end_x = (cap_power-16)*33+160;//cap_power+
	front_sight->end_y = 900;	
	front_sight->width = 4;
}

/**
 * @brief 客户端绘制5个图形配置
 * @author fwb
 * @note 绘制标尺  
 * @param 
 * @retval 
*/ 
void Front_sight_set1(ext_client_custom_graphic_five_t* front_sight)
{
	ShowRobotState();
	ScaleCtrl_Shoot_17mm();
	memset(front_sight,0,sizeof(ext_client_custom_graphic_five_t));
	front_sight->graphic_data_struct[0].graphic_name[0] = rand() % 255;
	front_sight->graphic_data_struct[0].graphic_name[1] = 0;
	front_sight->graphic_data_struct[0].graphic_name[2] = 0;
	front_sight->graphic_data_struct[0].operate_type=1;
	front_sight->graphic_data_struct[0].graphic_type=0;
	front_sight->graphic_data_struct[0].color=0;
	front_sight->graphic_data_struct[0].layer=1;
	front_sight->graphic_data_struct[0].start_x = 960;
	front_sight->graphic_data_struct[0].start_y = 500;
	front_sight->graphic_data_struct[0].end_x = 960;
	front_sight->graphic_data_struct[0].end_y = 150;	
	front_sight->graphic_data_struct[0].width = 2;

	front_sight->graphic_data_struct[1].graphic_name[0] = rand() % 255 ;
	front_sight->graphic_data_struct[1].graphic_name[1] = 1;
	front_sight->graphic_data_struct[1].graphic_name[2] = 0;
	front_sight->graphic_data_struct[1].operate_type=1;
	front_sight->graphic_data_struct[1].graphic_type=0;
	front_sight->graphic_data_struct[1].color=1;
	front_sight->graphic_data_struct[1].layer=1;
	front_sight->graphic_data_struct[1].start_x = 910;
	front_sight->graphic_data_struct[1].start_y = 425;//540
	front_sight->graphic_data_struct[1].end_x = 1010;
	front_sight->graphic_data_struct[1].end_y = 425;	//540
	front_sight->graphic_data_struct[1].width = 2;

	front_sight->graphic_data_struct[2].graphic_name[0] = rand() % 255 ;
	front_sight->graphic_data_struct[2].graphic_name[1] = 2;
	front_sight->graphic_data_struct[2].graphic_name[2] = 0;
	front_sight->graphic_data_struct[2].operate_type=1;
	front_sight->graphic_data_struct[2].graphic_type=0;
	front_sight->graphic_data_struct[2].color=1;
	front_sight->graphic_data_struct[2].layer=1;
	front_sight->graphic_data_struct[2].start_x = 900;
	front_sight->graphic_data_struct[2].start_y = Line3_y_pos;//510   
	front_sight->graphic_data_struct[2].end_x = 1020;
	front_sight->graphic_data_struct[2].end_y = Line3_y_pos;//510	
	front_sight->graphic_data_struct[2].width = 2;

	front_sight->graphic_data_struct[3].graphic_name[0] = 0 ;
	front_sight->graphic_data_struct[3].graphic_name[1] = 0;
	front_sight->graphic_data_struct[3].graphic_name[2] = 0;
	front_sight->graphic_data_struct[3].operate_type=1;
	front_sight->graphic_data_struct[3].graphic_type=0;
	front_sight->graphic_data_struct[3].color=1;
	front_sight->graphic_data_struct[3].layer=1;
	front_sight->graphic_data_struct[3].start_x = 890;
	front_sight->graphic_data_struct[3].start_y = Line4_y_pos;//480
	front_sight->graphic_data_struct[3].end_x = 1030;
	front_sight->graphic_data_struct[3].end_y = Line4_y_pos;//480	
	front_sight->graphic_data_struct[3].width = 2;

	front_sight->graphic_data_struct[4].graphic_name[0] = rand() % 255;
	front_sight->graphic_data_struct[4].graphic_name[1] = 0;
	front_sight->graphic_data_struct[4].graphic_name[2] = 0;
	front_sight->graphic_data_struct[4].operate_type=1;
	front_sight->graphic_data_struct[4].graphic_type=0;
	front_sight->graphic_data_struct[4].color=1;
	front_sight->graphic_data_struct[4].layer=1;
	front_sight->graphic_data_struct[4].start_x = 880;
	front_sight->graphic_data_struct[4].start_y = Line5_y_pos;//450
	front_sight->graphic_data_struct[4].end_x = 1040;
	front_sight->graphic_data_struct[4].end_y = Line5_y_pos;//450	
	front_sight->graphic_data_struct[4].width = 2;
}

/**
 * @brief 客户端绘制5个图形配置
 * @author fwb
 * @note 电容框 防撞线 自瞄指示   
 * @param 
 * @retval 
*/ 
void Armor_set(ext_client_custom_graphic_five_t* front_sight)
{
	memset(front_sight,0,sizeof(ext_client_custom_graphic_five_t));
	front_sight->graphic_data_struct[0].graphic_name[0] = 0; //电容框
	front_sight->graphic_data_struct[0].graphic_name[1] = 0;
	front_sight->graphic_data_struct[0].graphic_name[2] = rand() % 255;
	front_sight->graphic_data_struct[0].operate_type=1;
	front_sight->graphic_data_struct[0].graphic_type=1;
	front_sight->graphic_data_struct[0].color = 8;
	front_sight->graphic_data_struct[0].layer=3;
	front_sight->graphic_data_struct[0].start_x = 160;			//中心x 960 右为正
	front_sight->graphic_data_struct[0].start_y = 900;			//中心y 540 上为正
	front_sight->graphic_data_struct[0].end_x = 380;
	front_sight->graphic_data_struct[0].end_y = 860;	
	front_sight->graphic_data_struct[0].width = 4;

	front_sight->graphic_data_struct[1].graphic_name[0] = 0;//防撞线1
	front_sight->graphic_data_struct[1].graphic_name[1] = 0;
	front_sight->graphic_data_struct[1].graphic_name[2] = rand() % 255;
	front_sight->graphic_data_struct[1].operate_type=1;
	front_sight->graphic_data_struct[1].graphic_type=0;
	front_sight->graphic_data_struct[1].color = 8;//GimbalMode_light
	front_sight->graphic_data_struct[1].layer=3;
	front_sight->graphic_data_struct[1].start_x = 360;
	front_sight->graphic_data_struct[1].start_y = 50;//540
	front_sight->graphic_data_struct[1].end_x = 760;	//150//Endx[0]
	front_sight->graphic_data_struct[1].end_y = 150;//510	
	front_sight->graphic_data_struct[1].width = 2;
	
	front_sight->graphic_data_struct[2].graphic_name[0] = 0;//防撞线2
	front_sight->graphic_data_struct[2].graphic_name[1] = 0;
	front_sight->graphic_data_struct[2].graphic_name[2] = rand() % 255;
	front_sight->graphic_data_struct[2].operate_type=1;
	front_sight->graphic_data_struct[2].graphic_type=0;
	front_sight->graphic_data_struct[2].color=8;
	front_sight->graphic_data_struct[2].layer=3;
	front_sight->graphic_data_struct[2].start_x = 760;//Startx[0]
	front_sight->graphic_data_struct[2].start_y = 150;//510//180
	front_sight->graphic_data_struct[2].end_x = 1160;	//150//Endx[0]
	front_sight->graphic_data_struct[2].end_y = 150;//510	
	front_sight->graphic_data_struct[2].width = 2;
	
	front_sight->graphic_data_struct[3].graphic_name[0] = 0;//防撞线3
	front_sight->graphic_data_struct[3].graphic_name[1] = 0;
	front_sight->graphic_data_struct[3].graphic_name[2] = rand() % 255;
	front_sight->graphic_data_struct[3].operate_type=1;
	front_sight->graphic_data_struct[3].graphic_type=0;
	front_sight->graphic_data_struct[3].color=8;
	front_sight->graphic_data_struct[3].layer=3;
	front_sight->graphic_data_struct[3].start_x = 1160;//Startx[0]
	front_sight->graphic_data_struct[3].start_y = 150;//510//180
	front_sight->graphic_data_struct[3].end_x = 1560;	//150//Endx[0]
	front_sight->graphic_data_struct[3].end_y = 50;//510	
	front_sight->graphic_data_struct[3].width = 2;
  
	front_sight->graphic_data_struct[4].graphic_name[0] = 0;//自瞄指示
	front_sight->graphic_data_struct[4].graphic_name[1] = 0;
	front_sight->graphic_data_struct[4].graphic_name[2] = rand() % 255;
	front_sight->graphic_data_struct[4].operate_type=1;
	front_sight->graphic_data_struct[4].graphic_type=1;
	front_sight->graphic_data_struct[4].color = autoAimState_color;
	front_sight->graphic_data_struct[4].layer= 3;//7
	front_sight->graphic_data_struct[4].start_x = 770;			//中心x 960 右为正
	front_sight->graphic_data_struct[4].start_y = 400;			//中心y 540 上为正
	front_sight->graphic_data_struct[4].end_x = 1150;
	front_sight->graphic_data_struct[4].end_y = 650;	
	front_sight->graphic_data_struct[4].width = AimState_switch;
}

/**
 * @brief 客户端绘制5个图形配置
 * @author dzl
 * @note 状态指示灯 
 * @param 
 * @retval 
*/ 
void State_set(ext_client_custom_graphic_five_t* front_sight)
{
	memset(front_sight,0,sizeof(ext_client_custom_graphic_five_t));
	//Fri状态指示
	front_sight->graphic_data_struct[0].graphic_name[0] = 0;
	front_sight->graphic_data_struct[0].graphic_name[1] = rand() % 255;
	front_sight->graphic_data_struct[0].graphic_name[2] = 0;
	front_sight->graphic_data_struct[0].operate_type=1;
	front_sight->graphic_data_struct[0].graphic_type=2;
	front_sight->graphic_data_struct[0].color = ChassisState_color;
	front_sight->graphic_data_struct[0].layer = 7;
	front_sight->graphic_data_struct[0].width = 8;
	front_sight->graphic_data_struct[0].start_x = 150;
	front_sight->graphic_data_struct[0].start_y = 531;
	front_sight->graphic_data_struct[0].radius = 10;
	//Hatch状态指示
	front_sight->graphic_data_struct[1].graphic_name[0] = 0;
	front_sight->graphic_data_struct[1].graphic_name[1] = rand() % 255;
	front_sight->graphic_data_struct[1].graphic_name[2] = 0;
	front_sight->graphic_data_struct[1].operate_type=1;
	front_sight->graphic_data_struct[1].graphic_type=2;
	front_sight->graphic_data_struct[1].color = GimbalState_color;
	front_sight->graphic_data_struct[1].layer = 7;
	front_sight->graphic_data_struct[1].width = 8;
	front_sight->graphic_data_struct[1].start_x = 150;
	front_sight->graphic_data_struct[1].start_y = 581;
	front_sight->graphic_data_struct[1].radius = 10;
	//Gimbal状态指示
	front_sight->graphic_data_struct[2].graphic_name[0] = 0;
	front_sight->graphic_data_struct[2].graphic_name[1] = rand() % 255;
	front_sight->graphic_data_struct[2].graphic_name[2] = 0;
	front_sight->graphic_data_struct[2].operate_type=1;
	front_sight->graphic_data_struct[2].graphic_type=2;
	front_sight->graphic_data_struct[2].color = HatchState_color;
	front_sight->graphic_data_struct[2].layer = 7;
	front_sight->graphic_data_struct[2].width = 8;
	front_sight->graphic_data_struct[2].start_x = 150;
	front_sight->graphic_data_struct[2].start_y = 631;
	front_sight->graphic_data_struct[2].radius = 10;
	//Chassis状态指示
	front_sight->graphic_data_struct[3].graphic_name[0] = 0;
	front_sight->graphic_data_struct[3].graphic_name[1] = rand() % 255;
	front_sight->graphic_data_struct[3].graphic_name[2] = 0;
	front_sight->graphic_data_struct[3].operate_type=1;
	front_sight->graphic_data_struct[3].graphic_type=2;
	front_sight->graphic_data_struct[3].color = FirState_color;
	front_sight->graphic_data_struct[3].layer = 7;
	front_sight->graphic_data_struct[3].width = 8;
	front_sight->graphic_data_struct[3].start_x = 150;
	front_sight->graphic_data_struct[3].start_y = 681;
	front_sight->graphic_data_struct[3].radius = 10;
	//AtkEnergeBuff状态指示
	front_sight->graphic_data_struct[4].graphic_name[0] = 0;
	front_sight->graphic_data_struct[4].graphic_name[1] = rand() % 255;
	front_sight->graphic_data_struct[4].graphic_name[2] = 0;
	front_sight->graphic_data_struct[4].operate_type=1;
	front_sight->graphic_data_struct[4].graphic_type=2;
	front_sight->graphic_data_struct[4].color = AtkBuffState_color;
	front_sight->graphic_data_struct[4].layer = 7;
	front_sight->graphic_data_struct[4].width = 8;
	front_sight->graphic_data_struct[4].start_x = 150;
	front_sight->graphic_data_struct[4].start_y = 731;
	front_sight->graphic_data_struct[4].radius = 10;
}

void Delete_All_set(ext_client_custom_graphic_delete_t* front_sight)
{
	front_sight->operate_type = 2;//删除所有
	front_sight->layer =5;//无用
}
void Delete_layer_set(ext_client_custom_graphic_delete_t* front_sight,uint8_t layer)
{
	front_sight->operate_type = 1;//删除特定图layer
	front_sight->layer =layer;
}

void Chassis_State(ext_client_custom_graphic_character_t* front_sight)
{
	//底盘状态指示 字符
	memset(front_sight,0,sizeof(ext_client_custom_graphic_character_t));
	front_sight->graphic_data_struct.graphic_name[0] = rand() % 255;
	front_sight->graphic_data_struct.graphic_name[1] = 0;
	front_sight->graphic_data_struct.graphic_name[2] = 0;
	front_sight->graphic_data_struct.operate_type=1;
	front_sight->graphic_data_struct.graphic_type=7;
	front_sight->graphic_data_struct.color=8;
	front_sight->graphic_data_struct.layer=9;
	front_sight->graphic_data_struct.start_angle=20; //中心x 960 右为正
	front_sight->graphic_data_struct.end_angle=20; //中心y 540 上为正
	front_sight->graphic_data_struct.width=3;
	front_sight->graphic_data_struct.start_x = 180;	
	front_sight->graphic_data_struct.start_y = 540;
	memcpy(front_sight->data, ChassisMsg, sizeof(ChassisMsg));

}
void Gimbal_State(ext_client_custom_graphic_character_t* front_sight)
{
	//云台状态指示 字符
	memset(front_sight,0,sizeof(ext_client_custom_graphic_character_t));
	front_sight->graphic_data_struct.graphic_name[0] = rand() % 255;
	front_sight->graphic_data_struct.graphic_name[1] = 0;
	front_sight->graphic_data_struct.graphic_name[2] = 0;
	front_sight->graphic_data_struct.operate_type=1;
	front_sight->graphic_data_struct.graphic_type=7;
	front_sight->graphic_data_struct.color=8;
	front_sight->graphic_data_struct.layer=9;
	front_sight->graphic_data_struct.start_angle=20; //中心x 960 右为正
	front_sight->graphic_data_struct.end_angle=20; //中心y 540 上为正
	front_sight->graphic_data_struct.width=3;
	front_sight->graphic_data_struct.start_x = 180;	
	front_sight->graphic_data_struct.start_y = 590;
	memcpy(front_sight->data, GimbalMsg, sizeof(GimbalMsg));	
}
void Hatch_State(ext_client_custom_graphic_character_t* front_sight)
{	
	//弹舱盖状态指示 字符
	memset(front_sight,0,sizeof(ext_client_custom_graphic_character_t));
	front_sight->graphic_data_struct.graphic_name[0] = rand() % 255;
	front_sight->graphic_data_struct.graphic_name[1] = 0;
	front_sight->graphic_data_struct.graphic_name[2] = 0;
	front_sight->graphic_data_struct.operate_type=1;
	front_sight->graphic_data_struct.graphic_type=7;
	front_sight->graphic_data_struct.color=8;
	front_sight->graphic_data_struct.layer=9;
	front_sight->graphic_data_struct.start_angle=20; //中心x 960 右为正
	front_sight->graphic_data_struct.end_angle=20; //中心y 540 上为正
	front_sight->graphic_data_struct.width=3;
	front_sight->graphic_data_struct.start_x = 180;	
	front_sight->graphic_data_struct.start_y = 640;
	memcpy(front_sight->data, HatchMsg, sizeof(HatchMsg));
}
void Fri_State(ext_client_custom_graphic_character_t* front_sight)
{
	//摩擦轮状态指示 字符
	memset(front_sight,0,sizeof(ext_client_custom_graphic_character_t));
	front_sight->graphic_data_struct.graphic_name[0] = rand() % 255;
	front_sight->graphic_data_struct.graphic_name[1] = 0;
	front_sight->graphic_data_struct.graphic_name[2] = 0;
	front_sight->graphic_data_struct.operate_type=1;
	front_sight->graphic_data_struct.graphic_type=7;
	front_sight->graphic_data_struct.color=8;
	front_sight->graphic_data_struct.layer=9;
	front_sight->graphic_data_struct.start_angle=20; //中心x 960 右为正
	front_sight->graphic_data_struct.end_angle=20; //中心y 540 上为正
	front_sight->graphic_data_struct.width=3;
	front_sight->graphic_data_struct.start_x = 180;	
	front_sight->graphic_data_struct.start_y = 690;
	memcpy(front_sight->data, FriMsg, sizeof(FriMsg));
}

void SentryInteractiveSet(ext_robot_interactive_data_t* robot_inter_data)
{
	robot_inter_data->sentry_chassis_state = 1;
	robot_inter_data->sentry_fire_state = 2;
	robot_inter_data->sentry_gimbal_state = 3;

}
//////////////////////////////////////////////////////////////////////////
/*
*							--打包发送--
*/
//////////////////////////////////////////////////////////////////////////
/**
 * @brief 绘制一个图形
 * @author fwb
 * @note 画其它数量图形修改相应数据长度 结构体 内容id 并修改相应图形设置函数    
 * @param none
 * @retval none
*/ 
void Draw_One_Pic(uint16_t send_ID, uint16_t recv_ID,uint16_t CapChange)
{	
	uint8_t cap_power = 0;

	int index=0;
	uint8_t packed_front_sight[30];//总长度 见协议
	uint16_t cmd_id = 0x0301;	   //机器人信息交互用cmd_id
	frame_header_struct_t front_sight_header = {(uint8_t)0xA5,21,(uint8_t)1,0};//21为 总长度（30）-9

	append_CRC8_check_sum((uint8_t*)&front_sight_header,5);
	ext_student_interactive_header_data_t front_sight_data_header = {0x0101,send_ID,recv_ID};//0x0101为绘制一个的命令 后面为发送者与接收者
	graphic_data_struct_t front_sight_data;

	memset(&front_sight_data,0,sizeof(graphic_data_struct_t));

	switch (CapChange)
	{
	case 0:
		One_Pic_set(&front_sight_data );
		break;
	case 1:
		One_Pic_Change(&front_sight_data,cap_power);
		break;	
	default:
		break;
	}
	
	memcpy(packed_front_sight+index,&front_sight_header,sizeof(front_sight_header));
	index+=sizeof(front_sight_header);
	memcpy(packed_front_sight+index,(uint8_t*)&cmd_id,sizeof(cmd_id));
	index+=sizeof(cmd_id);
	memcpy(packed_front_sight+index,&front_sight_data_header,sizeof(front_sight_data_header));
	index+=sizeof(front_sight_data_header);
	memcpy(packed_front_sight+index,&front_sight_data,sizeof(front_sight_data));
	append_CRC16_check_sum(packed_front_sight,sizeof(packed_front_sight));

	HAL_UART_Transmit(&huart6,packed_front_sight,sizeof(packed_front_sight),20);
}

/**
 * @brief 绘制一个5个图形 如标尺
 * @author fwb
 * @note   
 * @param 
 * @retaval 
*/ 
void Draw_Interface(uint16_t send_ID, uint16_t recv_ID,uint16_t CapChange)
{
	int index=0;
	uint8_t packed_front_sight[90];
	uint16_t cmd_id = 0x0301;
	frame_header_struct_t front_sight_header = {(uint8_t)0xA5,81,(uint8_t)1,0};
	append_CRC8_check_sum((uint8_t*)&front_sight_header,5);
	ext_student_interactive_header_data_t front_sight_data_header = {0x0103,send_ID,recv_ID};
	ext_client_custom_graphic_five_t front_sight_data;
	memset(&front_sight_data,0,sizeof(ext_client_custom_graphic_five_t));	

	//选择绘制图形
	switch (CapChange)
	{
	case 0:
		Front_sight_set1(&front_sight_data);break;
	case 1:
		Armor_set(&front_sight_data);break;
	case 2:
		State_set(&front_sight_data);break;
	default:break;
	}
	memcpy(packed_front_sight+index,&front_sight_header,sizeof(front_sight_header));
	index+=sizeof(front_sight_header);
	memcpy(packed_front_sight+index,(uint8_t*)&cmd_id,sizeof(cmd_id));
	index+=sizeof(cmd_id);
	memcpy(packed_front_sight+index,&front_sight_data_header,sizeof(front_sight_data_header));
	index+=sizeof(front_sight_data_header);
	memcpy(packed_front_sight+index,&front_sight_data,sizeof(front_sight_data));
	append_CRC16_check_sum(packed_front_sight,sizeof(packed_front_sight));

	HAL_UART_Transmit(&huart6,packed_front_sight,sizeof(packed_front_sight),20);
}

/**
 * @brief 绘制一个字符
 * @author fwb
 * @note 用作其它绘制可能涉及修改相应数据长度 结构体 内容id 并修改相应图形设置函数    
 * @param none
 * @retval none
*/ 
void Draw_StateChar(uint16_t send_ID, uint16_t recv_ID,uint16_t CharChange)
{
	int index=0;
	uint8_t packed_front_sight[60];
	uint16_t cmd_id = 0x0301;
	frame_header_struct_t front_sight_header = {(uint8_t)0xA5,51,(uint8_t)1,0};

	append_CRC8_check_sum((uint8_t*)&front_sight_header,5);
	ext_student_interactive_header_data_t front_sight_data_header = {0x0110,send_ID,recv_ID};
	ext_client_custom_graphic_character_t front_sight_data;

	memset(&front_sight_data,0,sizeof(ext_client_custom_graphic_character_t));

	//选择绘制字符
	switch (CharChange)
	{
	case 0:
		Chassis_State(&front_sight_data);break;
	case 1:
		Gimbal_State(&front_sight_data);break;
	case 2:
		Hatch_State(&front_sight_data);break;	
	case 3:
		Fri_State(&front_sight_data);break;	
	default:break;
	}

	memcpy(packed_front_sight+index,&front_sight_header,sizeof(front_sight_header));
	index+=sizeof(front_sight_header);
	memcpy(packed_front_sight+index,(uint8_t*)&cmd_id,sizeof(cmd_id));
	index+=sizeof(cmd_id);
	memcpy(packed_front_sight+index,&front_sight_data_header,sizeof(front_sight_data_header));
	index+=sizeof(front_sight_data_header);
	memcpy(packed_front_sight+index,&front_sight_data,sizeof(front_sight_data));
	append_CRC16_check_sum(packed_front_sight,sizeof(packed_front_sight));

	HAL_UART_Transmit(&huart6,packed_front_sight,sizeof(packed_front_sight),20);
}


void Delete_Pic(uint16_t send_ID, uint16_t recv_ID,uint16_t DelChange)
{
	int index=0;
	uint8_t packed_front_sight1[17];
	uint16_t cmd_id = 0x0301;
	frame_header_struct_t front_sight_header = {(uint8_t)0xA5,8,(uint8_t)1,0};

	append_CRC8_check_sum((uint8_t*)&front_sight_header,5);
	ext_student_interactive_header_data_t front_sight_data_header = {0x0100,send_ID,recv_ID};
	ext_client_custom_graphic_delete_t front_sight_data;

	memset(&front_sight_data,0,sizeof(ext_client_custom_graphic_delete_t));

	if(DelChange == 0)
	Delete_All_set(&front_sight_data);
	else
	Delete_layer_set(&front_sight_data,DelChange);

	memcpy(packed_front_sight1+index,&front_sight_header,sizeof(front_sight_header));
	index+=sizeof(front_sight_header);
	memcpy(packed_front_sight1+index,(uint8_t*)&cmd_id,sizeof(cmd_id));
	index+=sizeof(cmd_id);
	memcpy(packed_front_sight1+index,&front_sight_data_header,sizeof(front_sight_data_header));
	index+=sizeof(front_sight_data_header);
	memcpy(packed_front_sight1+index,&front_sight_data,sizeof(front_sight_data));
	append_CRC16_check_sum(packed_front_sight1,sizeof(packed_front_sight1));

	HAL_UART_Transmit(&huart6,packed_front_sight1,sizeof(packed_front_sight1),20);
}

void Robot_Interactive(uint16_t send_ID, uint16_t recv_ID)
{
	int index=0;
	uint8_t packed_robot_inter_data[18];
	uint16_t cmd_id = 0x0301;
	frame_header_struct_t front_sight_header = {(uint8_t)0xA5,9,(uint8_t)1,0};

	append_CRC8_check_sum((uint8_t*)&front_sight_header,5);
	ext_student_interactive_header_data_t front_sight_data_header = {0x0200,send_ID,recv_ID};
	ext_robot_interactive_data_t robot_inter_data;

	memset(&robot_inter_data,0,sizeof(ext_robot_interactive_data_t));

	SentryInteractiveSet(&robot_inter_data);
	
	memcpy(packed_robot_inter_data+index,&front_sight_header,sizeof(front_sight_header));
	index+=sizeof(front_sight_header);
	memcpy(packed_robot_inter_data+index,(uint8_t*)&cmd_id,sizeof(cmd_id));
	index+=sizeof(cmd_id);
	memcpy(packed_robot_inter_data+index,&front_sight_data_header,sizeof(front_sight_data_header));
	index+=sizeof(front_sight_data_header);
	memcpy(packed_robot_inter_data+index,&robot_inter_data,sizeof(robot_inter_data));
	append_CRC16_check_sum(packed_robot_inter_data,sizeof(packed_robot_inter_data));

	HAL_UART_Transmit(&huart6,packed_robot_inter_data,sizeof(packed_robot_inter_data),20);
}

//主要界面绘制
void IsChange_Delete(void)
{
	uint16_t CoolingLimit;
	uint16_t Heat0;
	uint8_t SpeedLimit = 0;
	get_shooter_17mm_data(&CoolingLimit,&Heat0,&SpeedLimit);
	
	if((LastLimit != SpeedLimit)||(LastChassisState!=infantry.ChassisMode)||(LastGimbalState!=infantry.GimbalMode)
		||(LastServoState!=Remote.key.c)||(LastShootState!=infantry.ShootWay)||(LastAimState!=infantry.GameMode)||(LastFriState!=infantry.ShootMode)||(LastautoAimState!=Remote.key.f))
	{
		switch(robot_state.robot_id)
		{	case 1:
				self_ID = RED_HERO;
				self_client_ID = RED_HERO_CLIENT;
			break;	
			case 2:
				self_ID = RED_ENGINEER;
				self_client_ID = RED_ENGINEER;
			break;	
			case 3:
				self_ID = RED_STANDARD_1;
				self_client_ID = RED_STANDARD_1_CLIENT;
			break;
			case 4:
				self_ID = RED_STANDARD_2;
				self_client_ID = RED_STANDARD_2_CLIENT;
			break;
			case 5:
				self_ID = RED_STANDARD_3;
				self_client_ID = RED_STANDARD_3_CLIENT;
			break;
			case 6:
				self_ID = RED_AERIAL;
				self_client_ID = RED_AERIAL_CLIENT;
			case 101:
				self_ID = BLUE_HERO;
				self_client_ID = BLUE_HERO_CLIENT;
			break;	
			case 102:
				self_ID = BLUE_ENGINEER;
				self_client_ID = BLUE_ENGINEER_CLIENT;
			break;
			case 103:
				self_ID = BLUE_STANDARD_1;
				self_client_ID = BLUE_STANDARD_1_CLIENT;
			break;
			case 104:
				self_ID = BLUE_STANDARD_2;
				self_client_ID = BLUE_STANDARD_2_CLIENT;
			break;
			case 105:
				self_ID = BLUE_STANDARD_3;
				self_client_ID = BLUE_STANDARD_3_CLIENT;
			break;
			case 106:
				self_ID = BLUE_AERIAL;
				self_client_ID = BLUE_AERIAL_CLIENT;
			break;
		}
		
		Delete_Pic(self_ID,self_client_ID,1);
		Delete_Pic(self_ID,self_client_ID,2);
		Delete_Pic(self_ID,self_client_ID,3);
		Delete_Pic(self_ID,self_client_ID,7);
		Delete_Pic(self_ID,self_client_ID,9);
		
		Draw_Interface(self_ID,self_client_ID,0);
		Draw_Interface(self_ID,self_client_ID,1);
		Draw_Interface(self_ID,self_client_ID,2);
		//Draw_State_Show(self_ID,self_client_ID,0);
		//Draw_State_Show(self_ID,self_client_ID,1);
		Draw_StateChar(self_ID,self_client_ID,0);
		Draw_StateChar(self_ID,self_client_ID,1);
		Draw_StateChar(self_ID,self_client_ID,2);
		Draw_StateChar(self_ID,self_client_ID,3);
	}
	
	LastLimit = SpeedLimit;
	LastChassisState = infantry.ChassisMode;
	LastGimbalState = infantry.GimbalMode;
	LastShootState = infantry.ShootWay;
	LastServoState = Remote.key.c;
	LastAimState = infantry.GameMode;
	LastFriState = infantry.ShootMode;
	LastautoAimState = Remote.key.f;
}
//超级电容电压指示线绘制
void UpdateCapPower(void)
{
	switch(robot_state.robot_id)
		{	case 1:
				self_ID = RED_HERO;
				self_client_ID = RED_HERO_CLIENT;
			break;	
			case 2:
				self_ID = RED_ENGINEER;
				self_client_ID = RED_ENGINEER;
			break;	
			case 3:
				self_ID = RED_STANDARD_1;
				self_client_ID = RED_STANDARD_1_CLIENT;
			break;
			case 4:
				self_ID = RED_STANDARD_2;
				self_client_ID = RED_STANDARD_2_CLIENT;
			break;
			case 5:
				self_ID = RED_STANDARD_3;
				self_client_ID = RED_STANDARD_3_CLIENT;
			break;
			case 6:
				self_ID = RED_AERIAL;
				self_client_ID = RED_AERIAL_CLIENT;
			case 101:
				self_ID = BLUE_HERO;
				self_client_ID = BLUE_HERO_CLIENT;
			break;	
			case 102:
				self_ID = BLUE_ENGINEER;
				self_client_ID = BLUE_ENGINEER_CLIENT;
			break;
			case 103:
				self_ID = BLUE_STANDARD_1;
				self_client_ID = BLUE_STANDARD_1_CLIENT;
			break;
			case 104:
				self_ID = BLUE_STANDARD_2;
				self_client_ID = BLUE_STANDARD_2_CLIENT;
			break;
			case 105:
				self_ID = BLUE_STANDARD_3;
				self_client_ID = BLUE_STANDARD_3_CLIENT;
			break;
			case 106:
				self_ID = BLUE_AERIAL;
				self_client_ID = BLUE_AERIAL_CLIENT;
			break;
		}
	Delete_Pic(self_ID,self_client_ID,5);
	Draw_One_Pic(self_ID,self_client_ID,0);	
}

//////////////////////////////////////////////////////////////////////////
/*
*						--读取信息并修改相关参数--
*/
//////////////////////////////////////////////////////////////////////////
void ScaleCtrl_Shoot_17mm(void)
{	
	uint16_t CoolingLimit;
	uint16_t Heat0;
	uint8_t SpeedLimit = 0;
	
	get_shooter_17mm_data(&CoolingLimit,&Heat0,&SpeedLimit);
		if(SpeedLimit == 15) {
			Line3_y_pos = 405;Line4_y_pos = 365;Line5_y_pos = 340;
		}
		else if(SpeedLimit == 18) {
			Line3_y_pos = 410;Line4_y_pos = 400;Line5_y_pos = 381;
		}
		else if(SpeedLimit == 30) {
			Line3_y_pos = 465;Line4_y_pos = 455;Line5_y_pos = 448;
		}
		else{
			Line3_y_pos = 405;Line4_y_pos = 365;Line5_y_pos = 340;
		}
}

void ShowRobotState(void)
{
	if(Remote.key.x == 0 && Remote.key.f == 1)
		AimState_switch = 2;
	else
		AimState_switch = 0;

	if(infantry.GameMode == AutoAim)
		autoAimState_color = 2;
	else
		autoAimState_color = 8;	
	
	switch(infantry.ChassisMode)
	{
	case Run:
		ChassisMsg[8] = 'R';ChassisMsg[9] = 'U';ChassisMsg[10] = 'N';
		ChassisState_color = 2;
	break; //RUN
	case RotateRun:
		ChassisMsg[8] = 'R';ChassisMsg[9] = 'O';ChassisMsg[10] = 'T';
		ChassisState_color = 1;
	break; //ROTATE
	case Independent:
		ChassisMsg[8] = 'I';ChassisMsg[9] = 'N';ChassisMsg[10] = 'D';
		ChassisState_color = 6;
	break; //INDEPENDENT
	case ClimbSpeedAcc:
		ChassisMsg[8] = 'C';ChassisMsg[9] = 'L';ChassisMsg[10] = 'I';
		ChassisState_color = 5;
	break; //CLIMB
	default:
		ChassisMsg[8] = 'R';ChassisMsg[9] = 'U';ChassisMsg[10] = 'N';
		ChassisState_color = 2;
	break;
	}
	switch(infantry.GimbalMode)
	{
	case Follow_Encoder_Mode:	
		GimbalMsg[7] = 'E';GimbalMsg[8] = 'n';GimbalMsg[9] = 'c';GimbalMsg[10] = 'o';
	break; //follow encoder
	case Follow_Gyro_Mode: 		
		GimbalMsg[7] = 'G';GimbalMsg[8] = 'y';
		GimbalMsg[9] = 'r';GimbalMsg[10] = 'o';
		GimbalState_color = 2;
	break;	//follow gyro
	case Follow_Chassis: 	
		GimbalMsg[7] = 'C';GimbalMsg[8] = 'h';
		GimbalMsg[9] = 'a';GimbalMsg[10] = 's';
		GimbalState_color = 6;
	break;	//follow chassis
	default:					
		GimbalMsg[7] = 'G';GimbalMsg[8] = 'y';
		GimbalMsg[9] = 'r';GimbalMsg[10] = 'o';
		GimbalState_color = 2;
	break;
	}
		
//	if(infantry.ShootWay == SingleShoot)
//	{
//		single_width = 3;double_width = 0;trible_width = 0;		
//	}
//	else if(infantry.ShootWay == DoubleShoot)
//	{
//		single_width = 3;double_width = 3;trible_width = 0;
//	}
//	else if(infantry.ShootWay == TribleShoot)
//	{
//		single_width = 3;double_width = 3;trible_width = 3;
//	}
//	else if(infantry.ShootWay == ContinuousShoot)
//	{
//		single_width = 8;double_width = 8;trible_width = 8;
//	}
			
	if(Remote.key.c == 1)
	{		
		ServoMode_switch = 11;
		HatchMsg[6] = 'O';HatchMsg[7] = 'p';
		HatchMsg[8] = 'e';HatchMsg[9] = 'n';
		HatchState_color = 3;
	}
	else
	{
		ServoMode_switch = 3;
		HatchMsg[6] = ' ';HatchMsg[7] = ' ';
		HatchMsg[8] = ' ';HatchMsg[9] = ' ';
		HatchState_color = 2;
	}
 
	if(infantry.ShootMode == shoot_enabled)
	{ 
		FriMsg[4] = 'O';FriMsg[5] = 'N';FriMsg[6] = ' ';
		FirState_color = 2;
	}
	else
	{
		FriMsg[4] = 'O';FriMsg[5] = 'F';FriMsg[6] = 'F';
		FirState_color = 3;
	}
	if(Remote.key.g == 1 && (Remote.keyboard&Key_ECtrl) == 0)
		AtkBuffState_color = 0;
	else if(Remote.key.b == 1 && (Remote.keyboard&Key_ECtrl) == 0)
		AtkBuffState_color = 4;
	else
		AtkBuffState_color = 8;
}		

