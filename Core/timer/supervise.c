#include "supervise.h"

int lost_err =  0x0000;                   //外设监控错误标志。该变量共12位，每一位可作为一个外设是否丢失的标志位，一共可检测12个外设。0代表未丢失，1代表丢失
int lost_counter[DETECT_NUM] = {0};       //各个外设对应的递增计数器，如果向上溢出则说明

void SuperviseTaskHandle(void)
{
  for(int i = 0; i < DETECT_NUM; i++)     //依次扫描各个外设的递增计数器是否溢出
	{
		if(lost_counter[i] < 20)              //如果没有溢出则继续计数，并将丢失标志位置0
		{
			lost_counter[i]++;
			lost_err &= ~(1<<i);
		}
		else																	//如果计数向上溢出，将丢失标志位置1
		{
			lost_err |= (1<<i);
		}
	}
}

/**
	* @brief 重载机器人各个外设的递增计数器（喂狗）
	* @param None
	* @retval None
	*/
void LostCounterFeed(int index)
{
	lost_counter[index] = 0;
}

/**
	* @brief 判断机器人指定外设是否丢失
	* @param 外设编号
	* @retval 外设是否丢失：0代表未丢失，非0代表丢失
	*/
int Is_Error(int index)
{
	return (index&lost_err);
}

/**
	* @brief 判断机器人是否丢失重要外设
	* @param None
	* @retval 0代表未丢失，1代表丢失（布尔值）
	*/
int Is_Serious_Error(void)
{
  return Is_Error(1<<0);
}

int Is_Any_Error(void)
{
  return lost_err;
}
