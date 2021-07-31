/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "beep.h"
#include "supervise.h"
#include "BMI088driver.h"
#include "AttitudeResolve.h"
#include "pc_communicate.h"
#include "referee.h"
#include "fifo.h"
#include "super_cap.h"
#include "ui.h"
#include "robot.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern struct IMU_t bmi088;
extern struct Robot_t infantry;
extern fifo_s_t referee_fifo;
extern uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
extern int32_t pitchPowerSet;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ShootTask */
osThreadId_t ShootTaskHandle;
const osThreadAttr_t ShootTask_attributes = {
  .name = "ShootTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for GimbalTask */
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for RobotCMDTask */
osThreadId_t RobotCMDTaskHandle;
const osThreadAttr_t RobotCMDTask_attributes = {
  .name = "RobotCMDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for RefereeTask */
osThreadId_t RefereeTaskHandle;
const osThreadAttr_t RefereeTask_attributes = {
  .name = "RefereeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SuperCabTask */
osThreadId_t SuperCabTaskHandle;
const osThreadAttr_t SuperCabTask_attributes = {
  .name = "SuperCabTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED_Timer */
osTimerId_t LED_TimerHandle;
const osTimerAttr_t LED_Timer_attributes = {
  .name = "LED_Timer"
};
/* Definitions for Supervise_Timer */
osTimerId_t Supervise_TimerHandle;
const osTimerAttr_t Supervise_Timer_attributes = {
  .name = "Supervise_Timer"
};
/* Definitions for PCTrans_Timer */
osTimerId_t PCTrans_TimerHandle;
const osTimerAttr_t PCTrans_Timer_attributes = {
  .name = "PCTrans_Timer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osTimerId_t canBusTransmit_TimerHandle;
const osTimerAttr_t canBusTransmit_Timer_Attrtibutes = {
    .name = "canBusTransmit_Timer"
};

void canBusTransmitTimer(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartImuTask(void *argument);
void StartShootTask(void *argument);
void StartChassisTask(void *argument);
void StartGimbalTask(void *argument);
void StartRobotCMDTask(void *argument);
void StartRefereeTask(void *argument);
void StartSuperCabTask(void *argument);
void LedTimer(void *argument);
void SuperviseTimer(void *argument);
void PCTransTimer(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of LED_Timer */
  LED_TimerHandle = osTimerNew(LedTimer, osTimerPeriodic, NULL, &LED_Timer_attributes);

  /* creation of Supervise_Timer */
  Supervise_TimerHandle = osTimerNew(SuperviseTimer, osTimerPeriodic, NULL, &Supervise_Timer_attributes);

  /* creation of PCTrans_Timer */
  PCTrans_TimerHandle = osTimerNew(PCTransTimer, osTimerPeriodic, NULL, &PCTrans_Timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
    canBusTransmit_TimerHandle = osTimerNew(canBusTransmitTimer, osTimerPeriodic, NULL, &canBusTransmit_Timer_Attrtibutes);

	osTimerStart(LED_TimerHandle, 200);
	osTimerStart(Supervise_TimerHandle, 2);
	osTimerStart(PCTrans_TimerHandle, 7);
    osTimerStart(canBusTransmit_TimerHandle, 1);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(StartImuTask, NULL, &ImuTask_attributes);

  /* creation of ShootTask */
  ShootTaskHandle = osThreadNew(StartShootTask, NULL, &ShootTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);

  /* creation of GimbalTask */
  GimbalTaskHandle = osThreadNew(StartGimbalTask, NULL, &GimbalTask_attributes);

  /* creation of RobotCMDTask */
  RobotCMDTaskHandle = osThreadNew(StartRobotCMDTask, NULL, &RobotCMDTask_attributes);

  /* creation of RefereeTask */
  RefereeTaskHandle = osThreadNew(StartRefereeTask, NULL, &RefereeTask_attributes);

  /* creation of SuperCabTask */
  SuperCabTaskHandle = osThreadNew(StartSuperCabTask, NULL, &SuperCabTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  if(bmi088.accBiasFound)
		  Beep();
	  osDelay(140);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
	portTickType currentTime;
	currentTime = xTaskGetTickCount();
	IMU_init(&bmi088);
	while(BMI088_init()){}
  /* Infinite loop */
  for(;;)
  {
	imuDataHandle(&bmi088);				//BMI088数据解算
	LostCounterFeed(GYRO_INDEX);
	vTaskDelayUntil(&currentTime, 2);
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartShootTask */
/**
* @brief Function implementing the ShootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShootTask */
void StartShootTask(void *argument)
{
  /* USER CODE BEGIN StartShootTask */
  /* Infinite loop */
  portTickType currentTime;
	currentTime = xTaskGetTickCount();
  for(;;)
  {
    if(Remote.inputmode == RC_Remote)
		ShootCtrl_Remote();
	else if(Remote.inputmode == RC_MouseKey)
		ShootCtrl_MouseKey();
	ShootParamChange();
    //osDelay(2);
    vTaskDelayUntil(&currentTime, 2);
  }
  /* USER CODE END StartShootTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  portTickType currentTime;
	currentTime = xTaskGetTickCount();
  for(;;)
  {
	if(Remote.inputmode == RC_Remote)
		ChassisCtrl_Remote();
	else if(Remote.inputmode == RC_MouseKey)
		ChassisCtrl_MouseKey();
	chassis_distance_calc_task();
    ///CanTransmit_1234(&hcan2, 0,0,0,0);
	ChassisParamChange();
	//osDelay(2);
  vTaskDelayUntil(&currentTime, 2);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGimbalTask */
void StartGimbalTask(void *argument)
{
  /* USER CODE BEGIN StartGimbalTask */
  /* Infinite loop */
  portTickType currentTime;
	currentTime = xTaskGetTickCount();
	GIMBAL_InitArgument();
  for(;;)
  {
	if(Remote.inputmode == RC_Remote)
		GimbalCtrl_Remote();
	else if(Remote.inputmode == RC_MouseKey)
		GimbalCtrl_MouseKey();
    //CanTransmit_9_12(&hcan1, 0,0,0,0);
    //CanTransmit_9_12(&hcan2, 0,0,0,0);
	GimbalParamChange();
    //osDelay(2);
    vTaskDelayUntil(&currentTime, 2);
  }
  /* USER CODE END StartGimbalTask */
}

/* USER CODE BEGIN Header_StartRobotCMDTask */
/**
* @brief Function implementing the RobotCMDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobotCMDTask */
void StartRobotCMDTask(void *argument)
{
  /* USER CODE BEGIN StartRobotCMDTask */
	RobotParamInit();
  /* Infinite loop */
  for(;;)
  {
	RobotStateChange();
    osDelay(2);
  }
  /* USER CODE END StartRobotCMDTask */
}

/* USER CODE BEGIN Header_StartRefereeTask */
/**
* @brief Function implementing the RefereeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRefereeTask */
void StartRefereeTask(void *argument)
{
  /* USER CODE BEGIN StartRefereeTask */
	fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
	USART6_Init();
	Referee_Data_Init();
	//InitUI();
  /* Infinite loop */
  for(;;)
  {
	Referee_Unpack_FIFO_Data();
	IsChange_Delete();
	UpdateCapPower();
    osDelay(100);
  }
  /* USER CODE END StartRefereeTask */
}

/* USER CODE BEGIN Header_StartSuperCabTask */
/**
* @brief Function implementing the SuperCabTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSuperCabTask */
void StartSuperCabTask(void *argument)
{
  /* USER CODE BEGIN StartSuperCabTask */
  portTickType currentTime;
	currentTime = xTaskGetTickCount();
	SuperCap_init();
  /* Infinite loop */
  for(;;)
  {
	SuperCap_thread();
  vTaskDelayUntil(&currentTime, 2);
  }
  /* USER CODE END StartSuperCabTask */
}

/* LedTimer function */
void LedTimer(void *argument)
{
  /* USER CODE BEGIN LedTimer */
	static int LED_cnt = 0;
	switch(LED_cnt)
	{
	case 0: HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);break;
	case 1: HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);break;
	case 2: HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);break;
	}
	if(LED_cnt == 2)
		LED_cnt = 0;
	else
		LED_cnt ++;
  /* USER CODE END LedTimer */
}

/* SuperviseTimer function */
void SuperviseTimer(void *argument)
{
  /* USER CODE BEGIN SuperviseTimer */
	SuperviseTaskHandle();
    

  /* USER CODE END SuperviseTimer */
}

/* PCTransTimer function */
void PCTransTimer(void *argument)
{
  /* USER CODE BEGIN PCTransTimer */
	static int PC_TX_cnt = 0;
	PC_TX_cnt ++;
	if(PC_TX_cnt & 1){
		//PcDataTramsmit(Pan);
	}
	if(PC_TX_cnt == 200){
		PcDataTramsmit(Config);
	}
	else if(PC_TX_cnt == 400){
		PcDataTramsmit(Speed);
		PC_TX_cnt = 0;
	}
	
  /* USER CODE END PCTransTimer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void canBusTransmitTimer(void *argument)
{
    static uint8_t CAN_TX_CNT = 0;
    if (CAN_TX_CNT == 1)
    {
        CAN_TX_CNT = 0;
        if (infantry.WorkState == STOP)
        {
            CanTransmit_1234(&hcan1, 0, 0, 0, 0);
        }
        else
        {
            CanTransmit_1234(&hcan1, can1_motor_1.speed_pid.output, can1_motor_2.speed_pid.output, can1_motor_3.speed_pid.output, can1_motor_4.speed_pid.output);
        }
    }
    else if (CAN_TX_CNT == 0)
    {
        CAN_TX_CNT++;
        if (infantry.WorkState == STOP)
        {
            CanTransmit_5678(&hcan1, 0, 0, 0, 0);
        }
        else
        {
            CanTransmit_5678(&hcan1, can1_motor_5.speed_pid.output, can1_motor_6.speed_pid.output,pitchPowerSet,0);
        }
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
