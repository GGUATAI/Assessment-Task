#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"

static void Chassis_Powerlessness_Control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void Chassis_No_Move_Control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void Chassis_RC_Control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);

chassis_behaviour_e chassis_behaviour_mode = CHASSIS_POWERLESSNESS;

//判断"chassis_behaviour_mode"的模式
void Chassis_Behaviour_Mode_Set(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
    {
      return;
    }
  //遥控器右边推杆 设置模式
  if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
      chassis_behaviour_mode = CHASSIS_RC_CONTROL;
    }
  else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
      chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
  else if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
      chassis_behaviour_mode = CHASSIS_POWERLESSNESS;
    }

  //根据行为模式选择一个底盘控制模式
  if (chassis_behaviour_mode == CHASSIS_RC_CONTROL)
    {
      chassis_move_mode->chassis_mode = CHASSIS_RC_CONTROL_MODE;
    }
  else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
      chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE_MODE;
    }
  else if (chassis_behaviour_mode == CHASSIS_POWERLESSNESS)
    {
      chassis_move_mode->chassis_mode = CHASSIS_POWERLESSNESS_MODE;
    }
}

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
 */
void Chassis_Behaviour_Control_Set(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

  if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
      return;
    }
  if (chassis_behaviour_mode == CHASSIS_POWERLESSNESS)
    {
      Chassis_Powerlessness_Control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
  else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
      Chassis_No_Move_Control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
  else if (chassis_behaviour_mode == CHASSIS_RC_CONTROL)
    {
      Chassis_RC_Control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}


//底盘无力
static void Chassis_Powerlessness_Control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
      return;
    }
  *vx_set = 0.0f;
  *vy_set = 0.0f;
  *wz_set = 0.0f;
}


//底盘固定不动
static void Chassis_No_Move_Control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
      return;
    }
  *vx_set = 0.0f;
  *vy_set = 0.0f;
  *wz_set = 0.0f;
}

//遥控器数据的设定值转换速度值
static void Chassis_RC_Control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
      return;
    }
  *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_RC_CONTROL_SCALE;
  *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_RC_CONTROL_SCALE;
  *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_RC_CONTROL_SCALE;
  return;
}
