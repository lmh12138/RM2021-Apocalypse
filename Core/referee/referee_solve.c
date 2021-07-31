#include "referee.h"
#include <stdio.h>
#include <string.h>

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t 	game_state;
ext_game_result_t 	game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t 					field_event;
ext_supply_projectile_action_t 		supply_projectile_action_t;
//ext_supply_projectile_booking_t 	 supply_projectile_booking_t;
ext_referee_warning_t 				referee_warning_t;

ext_game_robot_state_t 	robot_state;
ext_power_heat_data_t 	power_heat_data_t;
ext_game_robot_pos_t 	game_robot_pos_t;
ext_buff_t 				buff_t;
aerial_robot_energy_t 	UAV_energy_t;
ext_robot_hurt_t 		robot_hurt_t;
ext_shoot_data_t 		shoot_data_t;
ext_bullet_remaining_t 	bullet_remaining_t;

ext_student_interactive_header_data_t student_inter_header_t;
ext_robot_interactive_data_t robot_inter_data_t;
ext_robot_command_t robot_command_t;
ext_client_map_command_t client_map_command_t;

/**
  * @brief          裁判系统数据初始化
  * @param[in]      none
  * @retval         none
  */
void Referee_Data_Init(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));

    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    //memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));

    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_t, 0, sizeof(ext_buff_t));
    memset(&UAV_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));
    memset(&student_inter_header_t, 0, sizeof(ext_student_interactive_header_data_t));
    memset(&robot_inter_data_t, 0, sizeof(ext_robot_interactive_data_t));
}

/**
  * @brief          裁判系统数据解析
  * @param[in]      裁判系统数据帧
  * @retval         true of false
  */
void Referee_Data_Solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;
    uint8_t index = 0;
	//取出帧头
    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);
	//取出cmd的id号
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(cmd_id);

    switch (cmd_id)
    {
	case GAME_STATE_CMD_ID:
		memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
		break;
	case GAME_RESULT_CMD_ID:
		memcpy(&game_result, frame + index, sizeof(game_result));
		break;
	case GAME_ROBOT_HP_CMD_ID:
		memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
		break;
	case FIELD_EVENTS_CMD_ID:
		memcpy(&field_event, frame + index, sizeof(field_event));
		break;
	case SUPPLY_PROJECTILE_ACTION_CMD_ID:
		memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
		break;
//	case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
//		memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
//		break;
	case REFEREE_WARNING_CMD_ID:
		memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
		break;
	case ROBOT_STATE_CMD_ID:
		memcpy(&robot_state, frame + index, sizeof(robot_state));
		break;
	case POWER_HEAT_DATA_CMD_ID:
		memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
		break;
	case ROBOT_POS_CMD_ID:
		memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
		break;
	case BUFF_MUSK_CMD_ID:
		memcpy(&buff_t, frame + index, sizeof(buff_t));
		break;
	case AERIAL_ROBOT_ENERGY_CMD_ID:
		memcpy(&UAV_energy_t, frame + index, sizeof(UAV_energy_t));
		break;
	case ROBOT_HURT_CMD_ID:
		memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
		break;
	case SHOOT_DATA_CMD_ID:
		memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
		break;
	case BULLET_REMAINING_CMD_ID:
		memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
		break;
	case STUDENT_INTERACTIVE_DATA_CMD_ID:
		memcpy(&student_inter_header_t, frame + index, sizeof(student_inter_header_t));
		memcpy(&robot_inter_data_t, frame+index+sizeof(student_inter_header_t), sizeof(robot_inter_data_t));
		break;
	default:
		break;
    }
}

/**
  * @brief          小弹丸热量数据
  * @param[in]      pointer of heat_limit_17mm
  * @param[in]      pointer of current_heat_17mm
  * @retval         none
  */
void get_shooter_17mm_data(uint16_t *cooling_limit_17mm, uint16_t *heat0, uint8_t *speed_limit_17mm)
{
    *cooling_limit_17mm = robot_state.shooter_id1_17mm_cooling_limit;
    *heat0 = power_heat_data_t.shooter_id1_17mm_cooling_heat;
    *speed_limit_17mm = robot_state.shooter_id1_17mm_speed_limit;
}

/**
  * @brief          底盘功率数据
  * @param[in]      pointer of power 功率
  * @param[in]      pointer of power buffer 缓冲功率
  * @retval         none
  */
void get_chassis_power_and_buffer(float *power, float *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;
}

/**
	* @brief          获取机器人ID
  * @param[in]      none
  * @retval         机器人ID
  */
uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

///**
//  * @brief          大弹丸热量数据
//  * @param[in]      pointer of heat_limit_42mm
//  * @param[in]      pointer of current_heat_42mm
//  * @retval         none
//  */
//void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
//{
//    *heat1_limit = robot_state.shooter_heat1_cooling_limit;
//    *heat1 = power_heat_data_t.shooter_heat1;
//}
