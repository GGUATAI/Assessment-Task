/*
 * bsp.usart.c
 *
 *  Created on: May 19, 2024
 *      Author: 25085
 */

#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
//遥控数据可视化
void Usart1_Tx_DMA_Init(void)//开启串口发送
{
    //使能DMA串口发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
}

void Usart1_Tx_DMA_Enable(uint8_t *data, uint16_t len)//开启串口DMA发送
{
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }
    //清除标志位（置零表示无传输完成或半传输完成中断）
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_HTIF7);
    //设置数据地址
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    //设置数据长度
    hdma_usart1_tx.Instance->NDTR = len;
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}



