/*
 * can_receive_transmit.h
 *
 *  Created on: May 20, 2024
 *      Author: 25085
 */

#ifndef INC_CAN_RECEIVE_TRANSMIT_H_
#define INC_CAN_RECEIVE_TRANSMIT_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "can.h"

#define CHASSIS_CAN hcan1

/* CAN send and receive ID */
typedef enum
{
  CAN_CHASSIS_ALL_ID = 0x200,
  CAN_3508_M1_ID = 0x201,
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  CAN_3508_M4_ID = 0x204,
} can_msg_id_e;

//rm motor data
typedef struct
{
  uint16_t ecd;//电机转子角度
  int16_t speed_rpm;//电机转子转速
  int16_t given_current;//控制电流/控制电压
  uint8_t temperate;//温度
  int16_t last_ecd;//上次电机转子角度
} motor_measure_t;


/**
 * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
extern void CAN_Cmd_Chassis_Reset_ID(void);
/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
extern void CAN_Cmd_Chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif /* INC_CAN_RECEIVE_TRANSMIT_H_ */
