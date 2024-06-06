/*
 * bsp.usart.h
 *
 *  Created on: May 19, 2024
 *      Author: 25085
 */

#ifndef INC_BSP_USART_H_
#define INC_BSP_USART_H_

#include "stm32f4xx_hal.h"
#include "main.h"

extern void Usart1_Tx_DMA_Init(void);
extern void Usart1_Tx_DMA_Enable(uint8_t *data, uint16_t len);//开启串口DMA发送

#endif /* INC_BSP_USART_H_ */
