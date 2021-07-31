#include "super_cap.h"
#include "string.h"
#include "referee.h"

CAN_TxHeaderTypeDef TxMessage_CAP = {
	.DLC=0x08,
	.StdId=0x210,
	.IDE=CAN_ID_STD,
	.RTR=CAN_RTR_DATA
};

extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;

struct SuperCap_t spuer_cap;
uint16_t super_cap_raw[4];
uint8_t chassis_power_limit;
uint16_t chassis_power_buffer;

void SuperCap_Encoder(struct SuperCap_t *cap, uint8_t *RecvData)
{
    memcpy(super_cap_raw, RecvData, sizeof(super_cap_raw));
    // cap->voltage_input_fdb = RecvData[0]<<8|RecvData[1];
    // cap->vlotage_cap_fdb = RecvData[2]<<8|RecvData[3];
    // cap->current_input_fdb = RecvData[4]<<8|RecvData[5];
    // cap->power_set_fdb = RecvData[6]<<8|RecvData[7];
    cap->voltage_input_fdb = (float)super_cap_raw[0] / 100.f;
    cap->vlotage_cap_fdb = (float)super_cap_raw[1] / 100.f;
    cap->current_input_fdb = (float)super_cap_raw[2] / 100.f;
    cap->power_set_fdb = (float)super_cap_raw[3] / 100.f;
}

void SuperCap_init(void)
{
    memset(&spuer_cap, 0, sizeof(spuer_cap));
    memset(super_cap_raw, 0, sizeof(super_cap_raw));
}

/**
	* @brief 
	* @param 
	* @retval None
	*/
void Can_tx_supercap(CAN_HandleTypeDef *hcanx, float power_set)
{
	uint16_t power_temp = power_set * 100;
	uint8_t TxData[8] = {0};
	TxData[0] = (uint8_t)(power_temp >> 8);
	TxData[1] = (uint8_t)(power_temp);

	uint32_t Can_TxMailbox;
	HAL_CAN_AddTxMessage(hcanx,&TxMessage_CAP,TxData, &Can_TxMailbox);
}

void SuperCap_thread(void)
{
	chassis_power_limit = robot_state.chassis_power_limit;
	chassis_power_buffer = power_heat_data_t.chassis_power_buffer; 

	if(chassis_power_buffer > 60 && chassis_power_buffer < 300)chassis_power_limit +=20;
	else if(chassis_power_buffer > 20)chassis_power_limit +=5;
    else 
    {
        chassis_power_limit -= 2;
    }

	if(chassis_power_limit < 45)chassis_power_limit = 45;
	Can_tx_supercap(&hcan2,chassis_power_limit);
}
