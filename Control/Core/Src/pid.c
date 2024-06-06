/*
 * pid.c
 *
 *  Created on: May 23, 2024
 *      Author: 25085
 */

#include "pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

static uint16_t BetaGeneration(float error,float epslion);//积分分离
static float VariableIntegralCoefficient(float error, float absmax,float absmin);//变积分
static float Abs_Limit(float parameter);//取绝对值，不改变传入值
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_INCREMENT: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      maxout_limit: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_Init(pid_type_def *pid, uint8_t mode, const float PID[3], float maxout_limit, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->increment_kp = PID[0];
    pid->increment_ki = PID[1];
    pid->increment_kd = PID[2];
    pid->maxout_limit = maxout_limit;
    pid->max_iout = max_iout;
    pid->d_error[0] = pid->d_error[1] = pid->d_error[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->kp_out = pid->ki_out = pid->kd_out = pid->now_result_out = 0.0f;
    pid->decide_front=0;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      current_value: 反馈数据
  * @param[in]      target_value: 设定值
  * @retval         pid输出
  */
float PID_Calc(pid_type_def *pid, float current_value, float target_value)
{
  if (pid == NULL)
    {
      return 0.0f;
    }

  pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->target_value = target_value;
  pid->current_value = current_value;
  pid->error[0] = target_value - current_value;
  uint16_t beta=BetaGeneration(pid->error[0], pid->espsilon);//积分分离
  float factor=VariableIntegralCoefficient(pid->error[0], pid->error_abs_max, pid->error_abs_min);
  if (pid->mode == PID_POSITION)
    {
      pid->kp_out = pid->increment_kp * pid->error[0];
      pid->ki_out += pid->increment_ki * pid->error[0];
      pid->d_error[2] = pid->d_error[1];
      pid->d_error[1] = pid->d_error[0];
      pid->d_error[0] = (pid->error[0] - pid->error[1]);
      pid->kd_out = pid->increment_kd * pid->d_error[0];
      LimitMax(pid->ki_out, pid->max_iout);
      pid->now_result_out = pid->kp_out + pid->ki_out + pid->kd_out;
      LimitMax(pid->now_result_out, pid->maxout_limit);
    }
  else if (pid->mode == PID_INCREMENT)
    {
      if(Abs_Limit(pid->error[0])>pid->deadband)
	{
	  if(pid->last_result_out>pid->maxout_limit)//抗积分饱和
	    {
	      if(pid->error<=0)
		{
		  pid->i_error=(pid->error[0]+pid->error[1])/2;//梯形积分
		}
	      else
		{
		  pid->i_error=0;
		}
	    }
	  else if(pid->last_result_out<pid->maxout_limit)
	    {
	      if(pid->error>=0)
		{
		  pid->i_error=(pid->error[0]+pid->error[1])/2;
		}
	      else
		{
		  pid->i_error=0;
		}
	    }
	  else
	    {
	      pid->i_error=(pid->error[0]+pid->error[1])/2;
	    }
	}
      else
	{
	  pid->i_error=0;
	}
      pid->kp_out = pid->increment_kp * (pid->error[0] - pid->error[1]);
      pid->ki_out = factor*pid->increment_ki * pid->i_error;//变积分
      pid->d_error[2] = pid->d_error[1];
      pid->d_error[1] = pid->d_error[0];
      pid->d_error[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
      if(pid->decide_front==1)
	{
	  float d1,d2,d3,temp;
	  temp=pid->gama*pid->increment_kd+pid->increment_kp;
	  d3=pid->increment_kd/temp;
	  d2=(pid->increment_kd+pid->increment_kp)/temp;
	  d1=pid->gama*d3;
	  pid->delta_current_value=pid->current_value-pid->last_current_value;
	  pid->kd_out=d1*pid->last_kd_out+d2*pid->delta_current_value+d3*pid->last_delta_current_value;//微分先行
	}
      pid->kd_out = (pid->increment_kd *(1-pid->alpha)* pid->d_error[0])+(pid->alpha*pid->last_kd_out);//不完全微分
      pid->last_kd_out=pid->kd_out;
      if(beta==1)
	{
	  pid->increment_out= pid->kp_out
	      +pid->ki_out
	      +pid->kd_out;
	}
      if(beta==0)
	{
	  pid->increment_out= pid->kp_out
	      +pid->kd_out;
	}
      pid->now_result_out=pid->increment_out+pid->last_result_out;
      pid->last_result_out=pid->now_result_out;
      LimitMax(pid->now_result_out, pid->maxout_limit);
    }
  return pid->now_result_out;
}

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_Clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->d_error[0] = pid->d_error[1] = pid->d_error[2] = 0.0f;
    pid->now_result_out = pid->kp_out = pid->ki_out = pid->kd_out = 0.0f;
    pid->current_value = pid->target_value = 0.0f;
}

static uint16_t BetaGeneration(float error,float epslion)//积分分离
{
  uint16_t beta=0;

  if(Abs_Limit(error)<=epslion)
    {
      beta=1;
    }

  return beta;
}

static float VariableIntegralCoefficient(float error, float absmax,float absmin)//absmax=A+B,absmin=B 变积分
{
  float factor=0.0;

  if(Abs_Limit(error)<=absmin)
    {
      factor=1.0;
    }
  else if(Abs_Limit(error)>absmax)
    {
      factor=0.0;
    }
  else
    {
      factor=(absmax-Abs_Limit(error))/(absmax-absmin);
    }

  return factor;
}

static float Abs_Limit(float parameter)//取绝对值，不改变传入值
{

  float par=0;

  if(parameter<0)
    {
      par=-parameter;
    }
  else
    {
      par=parameter;
    }

  return par;
}
