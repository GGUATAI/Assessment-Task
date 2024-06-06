/*
 * chassis_task.h
 *
 *  Created on: May 23, 2024
 *      Author: 25085
 */

#ifndef INC_CHASSIS_TASK_H_
#define INC_CHASSIS_TASK_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "math.h"
#include "can_receive_transmit.h"
#include "remote_control.h"
#include "pid.h"

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
//旋转的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 2
//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
//遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f
#define CHASSIS_WZ_SET_SCALE 0.1f
//底盘电机速度环PID
#define M3508_MOTOR_SPEED_PID_KP 1500.0f
#define M3508_MOTOR_SPEED_PID_KI 75.0f
#define M3508_MOTOR_SPEED_PID_KD 7.5f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


typedef enum
{
  CHASSIS_POWERLESSNESS_MODE,
  CHASSIS_NO_MOVE_MODE,
  CHASSIS_RC_CONTROL_MODE,
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float accel;
  float speed;
  float speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
  chassis_mode_e chassis_mode;               //底盘控制状态机
  chassis_motor_t motor_chassis[4];          //底盘电机数据
  pid_type_def motor_speed_pid[4];             //底盘电机速度pid

  float vx;                          //底盘速度 前进方向 前为正，单位 m/s
  float vy;                          //底盘速度 左右方向 左为正  单位 m/s
  float wz;                          //底盘旋转角速度，逆时针为正 单位 rad/s
  float vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
  float vy_set;                      //底盘设定速度 左右方向 左为正，单位 m/s
  float wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s

  float vx_max_speed;  //前进方向最大速度 单位m/s
  float vx_min_speed;  //后退方向最大速度 单位m/s
  float vy_max_speed;  //左方向最大速度 单位m/s
  float vy_min_speed;  //右方向最大速度 单位m/s
} chassis_move_t;

extern chassis_move_t chassis_move;

extern void Chassis_Task(void const *pvParameters);


#endif /* INC_CHASSIS_TASK_H_ */
