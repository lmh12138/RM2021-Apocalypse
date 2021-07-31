#ifndef IMU_HEAT_H_
#define IMU_HEAT_H_

#include "stm32f4xx.h"
#include "pid.h"
#include "cmsis_os.h"

#define HEAT_PID_KP 1600.0f //kp of temperature control PID 
#define HEAT_PID_KI 0.2f    //ki of temperature control PID 
#define HEAT_PID_KD 0.0f    //kd of temperature control PID 
#define HEAT_PID_MAX_OUT 4500.0f  //max out of temperature control PID 
#define HEAT_PID_MAX_IOUT 4400.0f  //max out of temperature control PID 

#define TIMx TIM10
#define CCRx CCR1
#define htimx htim10

extern void IMU_Heat_init(void);
extern void IMU_Heat_Control(float temp);

#endif
