/*
 * chassis_task.c
 *
 *  Created on: May 23, 2024
 *      Author: 25085
 */

#include "chassis_task.h"
#include "chassis_behaviour.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
  if ((input) > (dealine) || (input) < -(dealine)) \
  {                                                \
      (output) = (input);                          \
  }                                                \
  else                                             \
  {                                                \
      (output) = 0;                                \
  }                                                \
    }

//设置底盘控制模式，主要在'Chassis_Behaviour_Mode_Set'函数中改变
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//因底盘模式改变，参数改变
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//获得控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//控制循环，根据控制设定值，计算电机电流值，进行控制
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

//底盘运动数据
chassis_move_t chassis_move;

void Chassis_Init(chassis_move_t *chassis_move_init)
{
  if (chassis_move_init == NULL)
    {
      return;
    }
  //底盘速度环pid值
  const static float motor_speed_pid[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
  uint8_t i=0;
  //底盘开机状态为原始
  chassis_move_init->chassis_mode = CHASSIS_RC_CONTROL_MODE;
  //获取遥控器指针
  chassis_move_init->chassis_RC = get_remote_control_point();
  //获取底盘电机数据指针，初始化PID
  for (i = 0; i < 4; i++)
    {
      chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
      PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
    }
  //最大 最小速度
  chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
  chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

  chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
  chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
  //更新一下数据
  chassis_feedback_update(chassis_move_init);
}

//忘记怎么把任务独立在一个文件要怎么搞了，暂时就这样写了
void Chassis_Task(void const *pvParameters)
{
  //设置底盘控制模式
  chassis_set_mode(&chassis_move);
  //底盘数据更新
  chassis_feedback_update(&chassis_move);
  //底盘控制量设置
  chassis_set_contorl(&chassis_move);
  //底盘控制PID计算
  chassis_control_loop(&chassis_move);
  //发送控制电流
  CAN_Cmd_Chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
		  chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
}

//设置底盘控制模式，主要在'Chassis_Behaviour_Mode_Set'函数中改变
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
    {
      return;
    }
  Chassis_Behaviour_Mode_Set(chassis_move_mode);
}
//底盘测量数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
  if (chassis_move_update == NULL)
    {
      return;
    }
  uint8_t i = 0;
  for (i = 0; i < 4; i++)
    {
      //更新电机速度，加速度是速度的PID微分
      chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    }
  //更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
  chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
  chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
  chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
}
//设置底盘控制设置值, 三运动控制值是通过Chassis_Behaviour_Control_Set函数设置的
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
  if (chassis_move_control == NULL)
    {
      return;
    }
  float vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
  //获取三个控制设置值
  Chassis_Behaviour_Control_Set(&vx_set, &vy_set, &angle_set, chassis_move_control);
  if (chassis_move_control->chassis_mode == CHASSIS_RC_CONTROL_MODE)
    {
      chassis_move_control->vx_set = vx_set;
      chassis_move_control->vy_set = vy_set;
      chassis_move_control->wz_set = angle_set;
    }
  if (chassis_move_control->chassis_mode == CHASSIS_POWERLESSNESS_MODE)
    {
      chassis_move_control->vx_set = vx_set;
      chassis_move_control->vy_set = vy_set;
      chassis_move_control->wz_set = angle_set;
    }
  if (chassis_move_control->chassis_mode == CHASSIS_NO_MOVE_MODE)
    {
      chassis_move_control->vx_set = vx_set;
      chassis_move_control->vy_set = vy_set;
      chassis_move_control->wz_set = angle_set;
    }
}

/**
 * @brief          四个全向轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个全向轮速度
 */
static void chassis_vector_to_omni_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{

  wheel_speed[0] = -vx_set -vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
  wheel_speed[3] = -vx_set +vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

//控制循环，根据控制设定值，计算电机电流值，进行控制
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
  float max_vector = 0.0f, vector_rate = 0.0f;
  float temp = 0.0f;
  float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;

  //全向轮运动分解
  chassis_vector_to_omni_wheel_speed(chassis_move_control_loop->vx_set,
				     chassis_move_control_loop->vy_set,
				     chassis_move_control_loop->wz_set,
				     wheel_speed);
  if (chassis_move_control_loop->chassis_mode == CHASSIS_POWERLESSNESS_MODE)
    {
      chassis_move_control_loop->motor_chassis[i].give_current = 0;
      return;
    }
  //计算轮子控制最大速度，并限制其最大速度
  for (i = 0; i < 4; i++)
    {
      chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
      temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
      if (max_vector < temp)
	{
	  max_vector = temp;
	}
    }
  if (max_vector > MAX_WHEEL_SPEED)
    {
      vector_rate = MAX_WHEEL_SPEED / max_vector;
      for (i = 0; i < 4; i++)
	{
	  chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
	}
    }
  //计算pid
  for (i = 0; i < 4; i++)
    {
      PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
  //赋值电流值
  for (i = 0; i < 4; i++)
    {
      chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].now_result_out);
    }
}
