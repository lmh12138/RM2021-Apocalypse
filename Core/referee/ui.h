#ifndef UI_H
#define UI_H

#include "referee.h"
#include "protocol.h"
#include "CRC8_CRC16.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "super_cap.h"
 
 extern uint16_t self_ID;
 extern uint16_t self_client_ID;
 
 
  __packed struct ui_arc_t
{
    uint8_t name[3];
	uint32_t reserve1:6;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t center_x:11;	
	uint32_t center_y:11;	
	uint32_t reserve2:10;
	uint32_t half_x:11;	
	uint32_t half_y:11; 
};
typedef union __ui_arc_u
{
	__packed struct ui_arc_t config;
	graphic_data_struct_t graph_full;
}UI_Arc_t;

//todo:??????
  __packed struct ui_retangle_t
{
    uint8_t name[3];
	uint32_t reserve1:6;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t reserve2:18;
	uint32_t width:10;
	uint32_t start_x:11;	
	uint32_t start_y:11;	
	uint32_t reserve3:10;
	uint32_t end_x:11;	
	uint32_t end_y:11 ;
};
typedef union __ui_retangle_u
{
	__packed struct ui_retangle_t config;
	graphic_data_struct_t graph_full;
}UI_Rec_t;

typedef UI_Rec_t UI_Line_t;

__packed struct ui_float_t
{
    uint8_t name[3];
	uint32_t reserve1:6;
	uint32_t layer:4;
	uint32_t color:4;
    uint32_t size:9;
    uint32_t scale:9;   
    uint32_t width:10;
    uint32_t start_x:11;	
	uint32_t start_y:11;
    int32_t value;
};
typedef union __ui_float_u
{
    __packed struct ui_float_t config;
    graphic_data_struct_t graph_full;
}UI_Float_t;

__packed struct ui_char_t
{
    uint8_t name[3];
	uint32_t reserve1:6;
	uint32_t layer:4;
	uint32_t color:4;
    uint32_t size:9;
    uint32_t length:9;   
    uint32_t width:10;
    uint32_t start_x:11;	
	uint32_t start_y:11;
    char* p_str;    
};
typedef union __ui_char_u
{
    __packed struct ui_char_t config;
    graphic_data_struct_t graph_full;
}UI_Char_t;

typedef enum __graph_type
{
    line,
    retangle,
    circle,
    oval,
    arc,
    float_num,
    int_num,
    character
}UI_Graph_Type_t;
typedef enum __graph_operation
{
    do_nothing,
    add,
    change,
    delete
}UI_Graph_Oper_t;

enum UI_Color_e
{
    blue_or_red,
    yellow,
    green,
    orange,
    claret, 
    pink,
    cyan,   
    black,
    white
};
typedef enum
{
    RED_HERO_CLIENT         = 0x0101,
    RED_ENGINEER_CLIENT     = 0x0102,
    RED_STANDARD_1_CLIENT   = 0x0103,
    RED_STANDARD_2_CLIENT   = 0x0104,
    RED_STANDARD_3_CLIENT   = 0x0105,
    RED_AERIAL_CLIENT       = 0x0106,

    BLUE_HERO_CLIENT        = 0x0165,
    BLUE_ENGINEER_CLIENT    = 0x0166,
    BLUE_STANDARD_1_CLIENT  = 0x0167,
    BLUE_STANDARD_2_CLIENT  = 0x0168,
    BLUE_STANDARD_3_CLIENT  = 0x0169,
    BLUE_AERIAL_CLIENT      = 0x016A,
   
} client_id_t;

typedef enum
{
    DELETE_GRAPHIC      = 8,
    DRAW_ONE_GRAPHIC    = 15,
    DRAW_TWO_GRAPHIC    = 36,
    DRAW_FIVE_GRAPHIC   = 81,
    DRAW_SEVEN_GRAPHIC  = 111,
    DRAW_CHAR           =51
}interactive_data_len;

#define __UI_GRAPH_INIT(_graph_ptr_,_graph_type_)							\
do{																		\
	(_graph_ptr_)->graph_full.operate_type=0;						\
	(_graph_ptr_)->graph_full.graphic_type=(uint32_t)(_graph_type_);\
}while(0U)

#define __UI_GRAPH_OPERATE(_graph_ptr_,_operation_)\
do{                                            \
    (_graph_ptr_)->graph_full.operate_type=(uint32_t)(_operation_);\
}while(0U)

/*  end */

void One_Pic_set(graphic_data_struct_t* front_sight);
void Front_sight_set1(ext_client_custom_graphic_five_t* front_sight);

void Chassis_State(ext_client_custom_graphic_character_t* front_sight);
void Gimbal_State(ext_client_custom_graphic_character_t* front_sight);
void Hatch_State(ext_client_custom_graphic_character_t* front_sight);
void Fri_State(ext_client_custom_graphic_character_t* front_sight);

void Draw_Char_Chassis(uint16_t send_ID, uint16_t recv_ID,uint16_t CharChange);

void Draw_One_Pic(uint16_t send_ID, uint16_t recv_ID,uint16_t CapChange);
void Draw_Shoot_Scale(uint16_t send_ID, uint16_t recv_ID,uint16_t pic_type);
void Delete_Pic(uint16_t send_ID, uint16_t recv_ID,uint16_t DelChange);

void IsChange_Delete(void);
void UpdateCapPower(void);


void ShowRobotState(void);
void ScaleCtrl_Shoot_17mm(void);
void ScaleCtrl_Shoot_42mm(void);

unsigned char CRC8_Table(unsigned char *p, char counter);
     
#endif
