/*
 * pid.h
 *
 *  Created on: May 23, 2024
 *      Author: 25085
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "math.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_INCREMENT
};

typedef struct
{
    uint8_t mode;

    float target_value;//目标值
    float current_value;//当前值

    float increment_kp;//比例系数
    float increment_ki;//积分系数
    float increment_kd;//微分系数
    float alpha;//不完全微分系数
    float gama;//微分先行系数

    float error[3]; //误差项 0最新 1上一次 2上上次
    float i_error;//增量积分i单次误差
    float d_error[3];  //增量微分项 0最新 1上一次 2上上次
    float error_abs_max;//最大误差绝对值容忍
    float error_abs_min;//最小误差绝对值容忍

    float maxout_limit;//最大输出限幅
    float minout_limit;//最小输出限幅
    float max_iout; //最大积分输出限幅

    float increment_out;//增量输出
    float now_result_out;//PID结果输出
    float last_result_out;//上次PID结果输出

    float kp_out;//自适应，暂时用不到
    float ki_out;//当前增量积分i输出
    float kd_out;//当前增量微分d输出

    float espsilon;//偏差检测阈值
    float deadband;//死区
    float delta_current_value;//当前过程增量
    float last_delta_current_value;//上次过程增量
    float last_current_value;//当前值
    float last_kd_out;//上次增量微分d输出

    float decide_front;//超前控制选通值

} pid_type_def;

//初始化pid参数数据
extern void PID_Init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
//pid计算输出结果
extern float PID_Calc(pid_type_def *pid, float ref, float set);
//pid参数数据清零
extern void PID_Clear(pid_type_def *pid);


#endif /* INC_PID_H_ */
