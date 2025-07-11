#include "angle_control.h"
#include "pid_control.h"
#include "jy61p.h"

/* 角度控制全局实例 */
AngleControl_TypeDef angle_control;

/**
 * @brief 初始化角度控制系统
 * @retval None
 */
void AngleControl_Init(void)
{
    /* 初始化角度控制结构体 */
    angle_control.mode = ANGLE_CONTROL_DISABLED;
    angle_control.enabled = 0;
    angle_control.target_roll = 0.0f;
    angle_control.target_pitch = 0.0f;
    angle_control.target_yaw = 0.0f;
    angle_control.balance_sensitivity = 1.0f;
    angle_control.turn_sensitivity = 1.0f;
    
    /* 初始化角度PID控制器 */
    Angle_PID_Init();
    
    /* 设置初始目标值 */
    Angle_PID_SetTarget(ANGLE_ROLL, 0.0f);
    Angle_PID_SetTarget(ANGLE_PITCH, 0.0f);
    Angle_PID_SetTarget(ANGLE_YAW, 0.0f);
}

/**
 * @brief 启用角度控制
 * @retval None
 */
void AngleControl_Enable(void)
{
    angle_control.enabled = 1;
    
    /* 重置PID状态 */
    Angle_PID_Reset(ANGLE_ROLL);
    Angle_PID_Reset(ANGLE_PITCH);
    Angle_PID_Reset(ANGLE_YAW);
}

/**
 * @brief 禁用角度控制
 * @retval None
 */
void AngleControl_Disable(void)
{
    angle_control.enabled = 0;
    angle_control.mode = ANGLE_CONTROL_DISABLED;
    
    /* 停止所有电机 */
    PID_SetSpeed(PID_MOTOR_A, 0.0f);
    PID_SetSpeed(PID_MOTOR_B, 0.0f);
    
    /* 重置PID状态 */
    Angle_PID_Reset(ANGLE_ROLL);
    Angle_PID_Reset(ANGLE_PITCH);
    Angle_PID_Reset(ANGLE_YAW);
}

/**
 * @brief 设置角度控制模式
 * @param mode: 控制模式
 * @retval None
 */
void AngleControl_SetMode(AngleControlMode_TypeDef mode)
{
    angle_control.mode = mode;
    
    if (mode != ANGLE_CONTROL_DISABLED)
    {
        angle_control.enabled = 1;
    }
    else
    {
        AngleControl_Disable();
    }
}

/**
 * @brief 设置目标角度（全轴控制）
 * @param roll: 目标Roll角度(度)
 * @param pitch: 目标Pitch角度(度)
 * @param yaw: 目标Yaw角度(度)
 * @retval None
 */
void AngleControl_SetTarget(float roll, float pitch, float yaw)
{
    angle_control.target_roll = roll;
    angle_control.target_pitch = pitch;
    angle_control.target_yaw = yaw;
    
    Angle_PID_SetTarget(ANGLE_ROLL, roll);
    Angle_PID_SetTarget(ANGLE_PITCH, pitch);
    Angle_PID_SetTarget(ANGLE_YAW, yaw);
}

/**
 * @brief 设置平衡目标角度（主要用于Roll轴平衡控制）
 * @param target_roll: 目标Roll角度(度)
 * @retval None
 */
void AngleControl_SetBalance(float target_roll)
{
    angle_control.target_roll = target_roll;
    Angle_PID_SetTarget(ANGLE_ROLL, target_roll);
    
    if (angle_control.mode == ANGLE_CONTROL_DISABLED)
    {
        AngleControl_SetMode(ANGLE_CONTROL_BALANCE);
    }
}

/**
 * @brief 设置转向目标角度（主要用于Yaw轴转向控制）
 * @param target_yaw: 目标Yaw角度(度)
 * @retval None
 */
void AngleControl_SetTurn(float target_yaw)
{
    angle_control.target_yaw = target_yaw;
    Angle_PID_SetTarget(ANGLE_YAW, target_yaw);
    
    if (angle_control.mode == ANGLE_CONTROL_DISABLED)
    {
        AngleControl_SetMode(ANGLE_CONTROL_TURN);
    }
}

/**
 * @brief 设置控制灵敏度
 * @param balance_sens: 平衡控制灵敏度 (0.1 - 2.0)
 * @param turn_sens: 转向控制灵敏度 (0.1 - 2.0)
 * @retval None
 */
void AngleControl_SetSensitivity(float balance_sens, float turn_sens)
{
    /* 限制灵敏度范围 */
    if (balance_sens < 0.1f) balance_sens = 0.1f;
    if (balance_sens > 2.0f) balance_sens = 2.0f;
    if (turn_sens < 0.1f) turn_sens = 0.1f;
    if (turn_sens > 2.0f) turn_sens = 2.0f;
    
    angle_control.balance_sensitivity = balance_sens;
    angle_control.turn_sensitivity = turn_sens;
    
    /* 动态调整PID参数 */
    angle_pid_kp = ANGLE_PID_KP * balance_sens;
    angle_pid_ki = ANGLE_PID_KI * balance_sens;
    angle_pid_kd = ANGLE_PID_KD * balance_sens;
}

/**
 * @brief 重置角度控制系统
 * @retval None
 */
void AngleControl_Reset(void)
{
    /* 重置目标角度为当前角度 */
    angle_control.target_roll = Roll;
    angle_control.target_pitch = Pitch;
    angle_control.target_yaw = Yaw;
    
    /* 更新PID目标值 */
    Angle_PID_SetTarget(ANGLE_ROLL, Roll);
    Angle_PID_SetTarget(ANGLE_PITCH, Pitch);
    Angle_PID_SetTarget(ANGLE_YAW, Yaw);
    
    /* 重置PID状态 */
    Angle_PID_Reset(ANGLE_ROLL);
    Angle_PID_Reset(ANGLE_PITCH);
    Angle_PID_Reset(ANGLE_YAW);
}

/**
 * @brief 获取角度控制状态
 * @retval 角度控制结构体指针
 */
AngleControl_TypeDef* AngleControl_GetStatus(void)
{
    return &angle_control;
}
