/**
  ******************************************************************************
  * 文件名          : referee_usart.c
  * 创建时间        : 2020.01.01
  * 作者            : 方纬博
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.05.9
  * 修改人          : 方纬博
  ******************************************************************************
  * 1.本代码基于STM32F407IGH6TR开发，编译环境为Keil 5，基于FreeRTOS进行开发。
  * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
	* 3.本代码包含大量中文注释，请以UTF-8编码格式打开
	* 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
  ******************************************************************************
**/
#include "referee.h"
#include "main.h"
#include "cmsis_os.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"

extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
//USART6 缓存数组
uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
//FIFO 缓存
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

void USART6_Init()
{
    //使能DMA串口接收和发送
    SET_BIT(USART_Referee->CR3, USART_CR3_DMAR);
    SET_BIT(USART_Referee->CR3, USART_CR3_DMAT);

    __HAL_UART_ENABLE_IT(&huart_Referee, UART_IT_IDLE);					//使能空闲中断

    __HAL_DMA_DISABLE(&hdma_usart6_rx);										//disable DMA失效DMA
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
		{
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
		}
    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(usart6_buf[0]);	//memory buffer 1内存缓冲区1
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(usart6_buf[1]);	//memory buffer 2内存缓冲区2
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, USART_RX_BUF_LENGHT);	//data length数据长度
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);		//enable double memory buffer使能双缓冲区
    __HAL_DMA_ENABLE(&hdma_usart6_rx);										//enable DMA使能DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);										//disable DMA失效DMA
    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
		{
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
		}
    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
}

void USART6_TX_DMA_Enable(uint8_t *data, uint16_t len)
{
    __HAL_DMA_DISABLE(&hdma_usart6_tx);										//disable DMA失效DMA
    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
		{
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
		}
    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);
    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);
    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}

/**
	* @brief 串口空闲中断（中断回调）函数
	* @param 串口号
	* @retval None
	*/
void Referee_IDLE_Callback(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart_Referee);
        static uint16_t this_time_rx_len = 0;
				
        if ((huart_Referee.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart_Referee.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart_Referee.hdmarx);
            __HAL_DMA_SET_COUNTER(huart_Referee.hdmarx, USART_RX_BUF_LENGHT);
            huart_Referee.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart_Referee.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
            //detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(huart_Referee.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart_Referee.hdmarx);
            __HAL_DMA_SET_COUNTER(huart_Referee.hdmarx, USART_RX_BUF_LENGHT);
            huart_Referee.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart_Referee.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
            //detect_hook(REFEREE_TOE);
        }
    }
}

/**
  * @brief          single byte upacked 单字节解包
  * @param[in]      void
  * @retval         none
  */
void Referee_Unpack_FIFO_Data(void)
{
	uint8_t byte = 0;
	uint8_t sof = HEADER_SOF;
	unpack_data_t *p_obj = &referee_unpack_obj;

	while ( fifo_s_used(&referee_fifo) )
	{
		byte = fifo_s_get(&referee_fifo);
		switch(p_obj->unpack_step)
		{
		case STEP_HEADER_SOF:
		{
			if(byte == sof) {
				p_obj->unpack_step = STEP_LENGTH_LOW;
				p_obj->protocol_packet[p_obj->index++] = byte;
			}
			else {
			p_obj->index = 0;
			}
		}break;

		case STEP_LENGTH_LOW:
		{
			p_obj->data_len = byte;
			p_obj->protocol_packet[p_obj->index++] = byte;
			p_obj->unpack_step = STEP_LENGTH_HIGH;
		}break;

		case STEP_LENGTH_HIGH:
		{
			p_obj->data_len |= (byte << 8);
			p_obj->protocol_packet[p_obj->index++] = byte;

			if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
				p_obj->unpack_step = STEP_FRAME_SEQ;
			else {
				p_obj->unpack_step = STEP_HEADER_SOF;
				p_obj->index = 0;
			}
		}break;

		case STEP_FRAME_SEQ:
		{
			p_obj->protocol_packet[p_obj->index++] = byte;
			p_obj->unpack_step = STEP_HEADER_CRC8;
		}break;

		case STEP_HEADER_CRC8:
		{
			p_obj->protocol_packet[p_obj->index++] = byte;

			if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
			{
				if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
					p_obj->unpack_step = STEP_DATA_CRC16;
				else {
					p_obj->unpack_step = STEP_HEADER_SOF;
					p_obj->index = 0;
				}
			}
		}break;  

		case STEP_DATA_CRC16:
		{
			if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
				p_obj->protocol_packet[p_obj->index++] = byte;  
			if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
				p_obj->unpack_step = STEP_HEADER_SOF;
				p_obj->index = 0;
				if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
					Referee_Data_Solve(p_obj->protocol_packet);
			}
		}break;

		default:
		{
			p_obj->unpack_step = STEP_HEADER_SOF;
			p_obj->index = 0;
		}break;
		}
	}
}
