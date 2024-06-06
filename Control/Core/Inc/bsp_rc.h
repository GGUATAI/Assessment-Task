/*
 * bsp_rc.h
 *
 *  Created on: May 19, 2024
 *      Author: 25085
 */

#ifndef INC_BSP_RC_H_
#define INC_BSP_RC_H_

#include "stm32f4xx_hal.h"
#include "main.h"

extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_Unable(void);
extern void RC_Restart(uint16_t dma_buf_num);

#endif /* INC_BSP_RC_H_ */
