#include "ramp.h"

/**
	* @brief 斜坡初始化函数
	* @param None
	* @retval None
	*/
void RampInit(struct ramp_t *ramp, int32_t scale)
{
	ramp->count = 0;
	ramp->scale = scale;
	ramp->out = 0;
}

/**
	* @brief 斜坡计算函数
	* @param 斜坡结构体
	* @retval 0~1之间的浮点数
	*/
float RampCalc(struct ramp_t *ramp)
{
	if (ramp->scale <= 0)
		return 0;
	ramp->count++;
	if (ramp->count >= ramp->scale)
		ramp->count = ramp->scale;

	ramp->out = ramp->count / ((float)ramp->scale);
	return ramp->out;
}
/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
  * @attention  
  */
float RAMP_float( float final, float now, float ramp )
{
	  float buffer = 0;
	
	
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}
