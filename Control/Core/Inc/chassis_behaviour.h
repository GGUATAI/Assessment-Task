/*
 * chassis_behaviour.h
 *
 *  Created on: May 23, 2024
 *      Author: 25085
 */

#ifndef INC_CHASSIS_BEHAVIOUR_H_
#define INC_CHASSIS_BEHAVIOUR_H_

/**
 * DJI模板
    如果要添加一个新的行为模式
    1.添加一个新行为名字在 chassis_behaviour_e
    2. 实现一个新的函数 chassis_xxx_xxx_control(float *vx, float *vy, float *wz, chassis_move_t * chassis )
        "vx,vy,wz" 参数是底盘运动控制输入量
        第一个参数: 'vx' 通常控制纵向移动,正值 前进， 负值 后退
        第二个参数: 'vy' 通常控制横向移动,正值 左移, 负值 右移
        第三个参数: 'wz' 可能是角度控制或者旋转速度控制
        在这个新的函数, 你能给 "vx","vy",and "wz" 赋值想要的速度参数
    3.  在"chassis_behaviour_mode_set"这个函数中，添加新的逻辑判断，给chassis_behaviour_mode赋值成CHASSIS_XXX_XXX
        在函数最后，添加"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,然后选择一种底盘控制模式
        4种:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 云台和底盘的相对角度
        你可以命名成"xxx_angle_set"而不是'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 底盘的陀螺仪计算出的绝对角度
        你可以命名成"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'是速度控制， 'wz'是旋转速度控制
        CHASSIS_VECTOR_RAW : 使用'vx' 'vy' and 'wz'直接线性计算出车轮的电流值，电流值将直接发送到can 总线上
    4.  在"chassis_behaviour_control_set" 函数的最后，添加
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
**/

#include "stm32f4xx_hal.h"
#include "main.h"
#include "chassis_behaviour.h"
#include "can_receive_transmit.h"
#include "pid.h"
#include "chassis_task.h"

typedef enum
{
  CHASSIS_POWERLESSNESS,           //底盘电机电流控制值为 0，因电机的电流环控制表现为无力，推动底盘不存在抵抗力
  CHASSIS_NO_MOVE,                 //底盘电机速度控制值为 0，因电机的速度环控制表现为不移动，推动底盘存在抵抗力
  CHASSIS_RC_CONTROL,              //底盘移动速度和旋转速度均由遥控器决定应用于只需要底盘控制的场合
} chassis_behaviour_e;

#define CHASSIS_RC_CONTROL_SCALE 0.00757

extern chassis_move_t chassis_move;
/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  */
extern void Chassis_Behaviour_Mode_Set(chassis_move_t *chassis_move_mode);
/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,底盘所有信息.
  */
extern void Chassis_Behaviour_Control_Set(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);
extern void Chassis_Init(chassis_move_t *chassis_move_init);

#endif /* INC_CHASSIS_BEHAVIOUR_H_ */
