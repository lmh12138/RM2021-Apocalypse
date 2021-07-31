#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"
#include "usart.h"
#include "protocol.h"
#include "CRC8_CRC16.h"

#define USART_Referee USART6
#define huart_Referee huart6

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;

//比赛信息部分
typedef __packed struct //0x0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;

    uint64_t SyncTimeStamp;
} ext_game_state_t;

typedef __packed struct //0x0002
{
    uint8_t winner;
} ext_game_result_t;
typedef __packed struct //0x0003
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP; 
 uint16_t red_3_robot_HP; 
 uint16_t red_4_robot_HP; 
 uint16_t red_5_robot_HP; 
 uint16_t red_7_robot_HP; 
 uint16_t red_outpost_HP;
 uint16_t red_base_HP; 
 uint16_t blue_1_robot_HP;
 uint16_t blue_2_robot_HP; 
 uint16_t blue_3_robot_HP; 
 uint16_t blue_4_robot_HP; 
 uint16_t blue_5_robot_HP; 
 uint16_t blue_7_robot_HP; 
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
} ext_game_robot_HP_t;

typedef __packed struct //0x0004
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;

//事件信息部分
typedef __packed struct //0x0101
{
    uint32_t event_type;
} ext_event_data_t;
typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//0x0103

typedef __packed struct //0x0104
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;
typedef __packed struct //0x0105
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

//机器人信息部分
typedef __packed struct //0x0201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;

    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit; 
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit; 
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;

    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct //0x0202
{
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power; 
	uint16_t chassis_power_buffer; 
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t power_rune_buff;
} ext_buff_t;
typedef __packed struct //0x0205
{
    uint8_t attack_time;
} aerial_robot_energy_t;
typedef __packed struct //0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;
typedef __packed struct //0x0207
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;
typedef __packed struct //0x0208
{
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num; 
} ext_bullet_remaining_t;
typedef __packed struct //0x0209
{
  uint32_t rfid_status;
} ext_rfid_status_t;
typedef __packed struct //0x020A
{
  uint8_t dart_launch_opening_status;
  uint8_t dart_attack_target;
  uint16_t  target_change_time;
  uint8_t first_dart_speed;
  uint8_t second_dart_speed;
  uint8_t third_dart_speed;
  uint8_t fourth_dart_speed;
  uint16_t last_dart_launch_time;
  uint16_t operate_launch_cmd_time;
} ext_dart_cilent_cmd_t;

//交互部分

/*机器人ID：
1.英雄R 2.工程R 3/4/5.步兵R 6.空中R 7.哨兵R 9.雷达站R
101.英雄B 102.工程B 103/104/105.步兵B 106.空中B 107.哨兵B 109.雷达站B
客户端ID：
0x0101.英雄操作手客户端R
0x0102.工程操作手客户端R
0x0103/0x0104/0x0105.步兵操作手客户端R
0x0106.空中操作手客户端R
0x0165.英雄操作手客户端B
0x0166.工程操作手客户端B
0x0167/0x0168/0x0169.步兵操作手客户端步兵B
0x016A.空中操作手客户端B
学生机器人间通信 cmd_id 0x0301，内容ID:0x0200~0x02FF
*/
typedef __packed struct //0x0301
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;
typedef __packed struct
{
	//uint8_t data[112];
    uint8_t sentry_chassis_state;
    uint8_t sentry_gimbal_state;
    uint8_t sentry_fire_state;
} ext_robot_interactive_data_t;	//机器人间通信数据

//0x0302 自定义控制器

typedef __packed struct //0x0303
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
} ext_robot_command_t;	//小地图交互数据

//0x0304 通过图传串口发送的键鼠和遥控器数据

//绘图部分
typedef __packed struct
{ 
    uint8_t operate_type; 
    uint8_t layer;   
} ext_client_custom_graphic_delete_t;

typedef __packed struct
{ 
    uint8_t  graphic_name[3]; 
    uint32_t operate_type:3; 
    uint32_t graphic_type:3; 
    uint32_t layer:4; 
    uint32_t color:4; 
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10; 
    uint32_t start_x:11; 
    uint32_t start_y:11;
    uint32_t radius:10; 
    uint32_t end_x:11; 
    uint32_t end_y:11;
} graphic_data_struct_t;

typedef __packed struct
{ 
    graphic_data_struct_t graphic_data_struct[5];    
} ext_client_custom_graphic_five_t; //绘制5个图形

typedef __packed struct
{ 
    graphic_data_struct_t graphic_data_struct[7];    
} ext_client_custom_graphic_seven_t; //绘制7个图形
typedef __packed struct
{ 
    graphic_data_struct_t graphic_data_struct;
    uint8_t data[30];    
} ext_client_custom_graphic_character_t; //绘制字符

typedef __packed struct //0x0305
{
    uint16_t target_robot_ID;
    float target_position_x;
    float target_position_y;
} ext_client_map_command_t;

void USART6_Init(void);
void USART6_TX_DMA_Enable(uint8_t *data, uint16_t len);
void Referee_Data_Solve(uint8_t *frame);

extern void Referee_IDLE_Callback(void);
extern void Referee_Data_Init(void);
extern void Referee_Unpack_FIFO_Data(void);

extern void get_shooter_17mm_data(uint16_t *cooling_limit_17mm, uint16_t *heat0, uint8_t *speed_limit_17mm);
extern void get_chassis_power_and_buffer(float *power, float *buffer);
extern uint8_t get_robot_id(void);

#endif
