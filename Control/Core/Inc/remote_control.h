/*
 * remote_control.h
 *
 *  Created on: May 19, 2024
 *      Author: 25085
 */

#ifndef INC_REMOTE_CONTROL_H_
#define INC_REMOTE_CONTROL_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "string.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- Data Struct ------------------------------------- */
typedef  struct
{
         struct
        {
                int16_t ch[5];
                char s[2];
        } rc;

        /*留着以后用*/
//         struct
//        {
//                int16_t x;
//                int16_t y;
//                int16_t z;
//                uint8_t press_l;
//                uint8_t press_r;
//        } mouse;
//         struct
//        {
//                uint16_t v;
//        } key;
} RC_ctrl_t;


//遥控器初始化
extern void Remote_Control_Init(void);
//获取遥控器数据指针
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_Data_Error(void);
extern void Slove_RC_Lost(void);
extern void Slove_Data_Error(void);

#endif /* INC_REMOTE_CONTROL_H_ */
