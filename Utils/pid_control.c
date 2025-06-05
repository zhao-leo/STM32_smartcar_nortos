#include "pid_control.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>

/* 全局PID参数，可用于调整电机A的PID控制器 */
float pid_kp_a = PID_KP_A;
float pid_ki_a = PID_KI_A;
float pid_kd_a = PID_KD_A;

/* 全局PID参数，可用于调整电机B的PID控制器 */
float pid_kp_b = PID_KP_B;
float pid_ki_b = PID_KI_B;
float pid_kd_b = PID_KD_B;

/* 定义电机PID控制器实例 */
PID_TypeDef pid_motor_a;  // 电机A的PID控制器
PID_TypeDef pid_motor_b;  // 电机B的PID控制器

/* 外部变量 */
extern Encoder_TypeDef encoderA;
extern Encoder_TypeDef encoderB;

/**
 * @brief 初始化PID控制器
 * @param pid: PID控制器结构体指针
 * @retval None
 */
void PID_Init(PID_TypeDef *pid)
{
    /* 根据控制器类型设置PID参数 */
    if (pid == &pid_motor_a) {
        pid->Kp = pid_kp_a;
        pid->Ki = pid_ki_a;
        pid->Kd = pid_kd_a;
    } else if (pid == &pid_motor_b) {
        pid->Kp = pid_kp_b;
        pid->Ki = pid_ki_b;
        pid->Kd = pid_kd_b;
    }
    
    /* 重置PID状态 */
    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    
    /* 设置输出限制 */
    pid->output_min = PID_OUTPUT_MIN;
    pid->output_max = PID_OUTPUT_MAX;
    
    /* 默认方向为正向 */
    pid->direction = MOTOR_FORWARD;
}

/**
 * @brief 计算PID控制器的输出
 * @param pid: PID控制器结构体指针
 * @retval PID输出值
 */
float PID_Compute(PID_TypeDef *pid)
{
    /* 计算当前误差 */
    pid->error = pid->setpoint - pid->feedback;
    
    /* 计算积分项 */
    pid->integral += pid->error;
    
    /* 积分项限幅，防止积分饱和 */
    if (pid->integral > (pid->output_max / pid->Ki))
        pid->integral = pid->output_max / pid->Ki;
    else if (pid->integral < (pid->output_min / pid->Ki))
        pid->integral = pid->output_min / pid->Ki;
    
    /* 计算微分项 (使用误差的变化率) */
    pid->derivative = pid->error - pid->last_error;
    
    /* 计算PID输出 */
    pid->output = pid->Kp * pid->error + 
                  pid->Ki * pid->integral + 
                  pid->Kd * pid->derivative;
    
    /* 输出限幅 */
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;
    
    /* 更新误差记录 */
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;
    
    /* 确定电机方向 */
    pid->direction = (pid->error >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    
    return pid->output;
}

/**
 * @brief 设置目标电机转速
 * @param motor_id: 电机ID (1-电机A, 2-电机B)
 * @param target_rpm: 目标转速(RPM)
 * @retval None
 */
void PID_SetSpeed(PID_Motor_ID motor_id, float target_rpm)
{
    if (motor_id == PID_MOTOR_A)
    {
        pid_motor_a.setpoint = target_rpm;
        
        /* 如果设置为0，直接停止电机，避免不必要的抖动 */
        if (target_rpm == 0)
        {
            Motor_Stop(1);
            pid_motor_a.integral = 0; // 重置积分项
        }
    }
    else if (motor_id == PID_MOTOR_B)
    {
        pid_motor_b.setpoint = target_rpm;
        
        /* 如果设置为0，直接停止电机，避免不必要的抖动 */
        if (target_rpm == 0)
        {
            Motor_Stop(2);
            pid_motor_b.integral = 0; // 重置积分项
        }
    }
}

/**
 * @brief 更新PID控制器并控制电机
 * @note 此函数应定期调用，与编码器更新频率一致
 * @retval None
 */
void PID_Update(void)
{
    uint16_t pwm_value;
    
    /* 更新PID控制器的反馈值 */
    pid_motor_a.feedback = encoderA.speed_rpm;
    pid_motor_b.feedback = encoderB.speed_rpm;

    /* 检查编码器反馈值，如果为零则重置PID积分项和微分项 */
    if (fabsf(encoderA.speed_rpm) < 0.1f) {  // 使用小阈值判断是否为零
        pid_motor_a.integral = 0.0f;
        pid_motor_a.derivative = 0.0f;
    }
    
    if (fabsf(encoderB.speed_rpm) < 0.1f) {  // 使用小阈值判断是否为零
        pid_motor_b.integral = 0.0f;
        pid_motor_b.derivative = 0.0f;
    }
    
    /* 更新PID控制器参数（以防被动态修改） */
    pid_motor_a.Kp = pid_kp_a;
    pid_motor_a.Ki = pid_ki_a;
    pid_motor_a.Kd = pid_kd_a;
    
    pid_motor_b.Kp = pid_kp_b;
    pid_motor_b.Ki = pid_ki_b;
    pid_motor_b.Kd = pid_kd_b;
    
    /* 计算电机A的PID控制值并应用 */
    if (pid_motor_a.setpoint != 0)
    {
        PID_Compute(&pid_motor_a);
        pwm_value = (uint16_t)fabsf(pid_motor_a.output);
        Motor_SetSpeed(1, pwm_value, (pid_motor_a.setpoint >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD);
    }
    
    /* 计算电机B的PID控制值并应用 */
    if (pid_motor_b.setpoint != 0)
    {
        PID_Compute(&pid_motor_b);
        pwm_value = (uint16_t)fabsf(pid_motor_b.output);
        Motor_SetSpeed(2, pwm_value, (pid_motor_b.setpoint >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD);
    }
}