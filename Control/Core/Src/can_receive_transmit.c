/*
 * can_receive_transmit.c
 *
 *  Created on: May 20, 2024
 *      Author: 25085
 */

#include "can_receive_transmit.h"



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
  (ptr)->last_ecd = (ptr)->ecd;                                   \
  (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
  (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
  (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
  (ptr)->temperate = (data)[6];                                   \
    }
/*电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;*/
static motor_measure_t motor_chassis[4];

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t  chassis_can_send_data[8];

/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  switch (rx_header.StdId)
  {
    case CAN_3508_M1_ID:
      {
	static uint8_t i = 0;
	//get motor id
	i = rx_header.StdId - CAN_3508_M1_ID;
	get_motor_measure(&motor_chassis[i], rx_data);
	break;
	//底盘电机1
      }
    case CAN_3508_M2_ID:
      {
	static uint8_t i = 0;
	//get motor id
	i = rx_header.StdId - CAN_3508_M1_ID;
	get_motor_measure(&motor_chassis[i], rx_data);
	break;
	//底盘电机2
      }
    case CAN_3508_M3_ID:
      {
	static uint8_t i = 0;
	//get motor id
	i = rx_header.StdId - CAN_3508_M1_ID;
	get_motor_measure(&motor_chassis[i], rx_data);
	break;
	//底盘电机3
      }
    case CAN_3508_M4_ID:
      {
	static uint8_t i = 0;
	//get motor id
	i = rx_header.StdId - CAN_3508_M1_ID;
	get_motor_measure(&motor_chassis[i], rx_data);
	break;
	//底盘电机4
      }
    default:
      {
	break;
      }
  }
}


/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void CAN_Cmd_Chassis_Reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN_Cmd_Chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}

