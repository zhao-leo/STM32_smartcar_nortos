#include "pid_control.h"
#include "encoder.h"
#include "motor.h"
#include "jy61p.h"
#include <math.h>

/* 全局PID参数，可用于调整电机A的PID控制器 */
float pid_kp_a = PID_KP_A;
float pid_ki_a = PID_KI_A;
float pid_kd_a = PID_KD_A;

/* 全局PID参数，可用于调整电机B的PID控制器 */
float pid_kp_b = PID_KP_B;
float pid_ki_b = PID_KI_B;
float pid_kd_b = PID_KD_B;

/* 角度环PID参数 */
float angle_pid_kp = ANGLE_PID_KP;
float angle_pid_ki = ANGLE_PID_KI;
float angle_pid_kd = ANGLE_PID_KD;

/* 定义电机PID控制器实例 */
PID_TypeDef pid_motor_a;  // 电机A的PID控制器
PID_TypeDef pid_motor_b;  // 电机B的PID控制器

/* 定义角度环PID控制器实例（仅保留Yaw轴） */
PID_TypeDef pid_angle_yaw;    // Yaw轴角度PID控制器

/* 外部变量 */
extern Encoder_TypeDef encoderA;
extern Encoder_TypeDef encoderB;

/* 控制状态全局变量 */
PID_ControlState pid_control_state = {
    .mode = PID_MODE_MANUAL,
    .base_speed = 0.0f,
    .target_yaw = 0.0f,
    .angle_control_active = 0
};

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
 * @param target_rpm: 目标转速(RPM) - 支持正负值
 * @retval None
 */
void PID_SetSpeed(PID_Motor_ID motor_id, float target_rpm)
{
    if (motor_id == PID_MOTOR_A)
    {
        pid_motor_a.setpoint = target_rpm;
        
        /* 如果设置为0，直接停止电机，避免不必要的抖动 */
        if (fabsf(target_rpm) < 0.1f)
        {
            Motor_Stop(1);
            pid_motor_a.integral = 0; // 重置积分项
        }
    }
    else if (motor_id == PID_MOTOR_B)
    {
        pid_motor_b.setpoint = target_rpm;
        
        /* 如果设置为0，直接停止电机，避免不必要的抖动 */
        if (fabsf(target_rpm) < 0.1f)
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
    uint8_t motor_direction;
    
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
    if (fabsf(pid_motor_a.setpoint) > 0.1f)
    {
        PID_Compute(&pid_motor_a);
        pwm_value = (uint16_t)fabsf(pid_motor_a.output);
        
        /* 根据设定值的正负确定方向 */
        motor_direction = (pid_motor_a.setpoint >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
        Motor_SetSpeed(1, pwm_value, motor_direction);
    }
    
    /* 计算电机B的PID控制值并应用 */
    if (fabsf(pid_motor_b.setpoint) > 0.1f)
    {
        PID_Compute(&pid_motor_b);
        pwm_value = (uint16_t)fabsf(pid_motor_b.output);
        
        /* 根据设定值的正负确定方向 */
        motor_direction = (pid_motor_b.setpoint >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
        Motor_SetSpeed(2, pwm_value, motor_direction);
    }
}

/**
 * @brief 角度差值计算（处理角度环绕问题）
 * @param target: 目标角度
 * @param current: 当前角度
 * @retval 角度差值
 */
static float Angle_Difference(float target, float current)
{
    float diff = target - current;
    
    /* 处理角度环绕问题，确保差值在-180到180度之间 */
    if (diff > 180.0f)
        diff -= 360.0f;
    else if (diff < -180.0f)
        diff += 360.0f;
    
    return diff;
}

/**
 * @brief 初始化角度环PID控制器（仅Yaw轴）
 * @retval None
 */
void Angle_PID_Init(void)
{
    /* 初始化Yaw轴PID控制器 */
    pid_angle_yaw.Kp = angle_pid_kp;
    pid_angle_yaw.Ki = angle_pid_ki;
    pid_angle_yaw.Kd = angle_pid_kd;
    pid_angle_yaw.setpoint = 0.0f;
    pid_angle_yaw.feedback = 0.0f;
    pid_angle_yaw.error = 0.0f;
    pid_angle_yaw.last_error = 0.0f;
    pid_angle_yaw.prev_error = 0.0f;
    pid_angle_yaw.integral = 0.0f;
    pid_angle_yaw.derivative = 0.0f;
    pid_angle_yaw.output = 0.0f;
    pid_angle_yaw.output_min = ANGLE_PID_OUTPUT_MIN;
    pid_angle_yaw.output_max = ANGLE_PID_OUTPUT_MAX;
}

/**
 * @brief 设置角度环目标值（支持Yaw轴）可以拓展为其余三轴
 * @param axis: 角度轴 (支持ANGLE_YAW)
 * @param target_angle: 目标角度(度)
 * @retval None
 */
void Angle_PID_SetTarget(PID_Angle_Axis axis, float target_angle)
{
    if(axis == ANGLE_YAW)
    {
        pid_angle_yaw.setpoint = target_angle;
    }
}

/**
 * @brief 重置角度环PID状态（仅支持Yaw轴）
 * @param axis: 角度轴 (仅支持ANGLE_YAW)
 * @retval None
 */
void Angle_PID_Reset(PID_Angle_Axis axis)
{
    if(axis == ANGLE_YAW)
    {
        pid_angle_yaw.error = 0.0f;
        pid_angle_yaw.last_error = 0.0f;
        pid_angle_yaw.prev_error = 0.0f;
        pid_angle_yaw.integral = 0.0f;
        pid_angle_yaw.derivative = 0.0f;
        pid_angle_yaw.output = 0.0f;
    }
}

/**
 * @brief 更新角度环PID控制器（仅Yaw轴）
 * @note 此函数应在定时器中定期调用，建议调用频率与陀螺仪读取频率一致
 * @retval None
 */
void Angle_PID_Update(void)
{
    float yaw_output;
    float motor_a_speed, motor_b_speed;
    
    /* 如果角度控制未激活，则直接返回 */
    if (!pid_control_state.angle_control_active) {
        return;
    }
    
    /* 更新Yaw轴PID参数 */
    pid_angle_yaw.Kp = angle_pid_kp;
    pid_angle_yaw.Ki = angle_pid_ki;
    pid_angle_yaw.Kd = angle_pid_kd;
    
    /* 更新反馈值 */
    pid_angle_yaw.feedback = Yaw;
    
    /* 计算角度误差 */
    pid_angle_yaw.error = Angle_Difference(pid_angle_yaw.setpoint, pid_angle_yaw.feedback);
    
    /* 计算积分项 */
    pid_angle_yaw.integral += pid_angle_yaw.error;
    
    /* 积分项限幅，防止积分饱和 */
    float integral_limit = pid_angle_yaw.output_max / pid_angle_yaw.Ki;
    if (pid_angle_yaw.integral > integral_limit)
        pid_angle_yaw.integral = integral_limit;
    else if (pid_angle_yaw.integral < -integral_limit)
        pid_angle_yaw.integral = -integral_limit;
    
    /* 计算微分项 */
    pid_angle_yaw.derivative = pid_angle_yaw.error - pid_angle_yaw.last_error;
    
    /* 计算PID输出 */
    yaw_output = pid_angle_yaw.Kp * pid_angle_yaw.error + 
                 pid_angle_yaw.Ki * pid_angle_yaw.integral + 
                 pid_angle_yaw.Kd * pid_angle_yaw.derivative;
    
    /* 输出限幅 */
    if (yaw_output > pid_angle_yaw.output_max)
        yaw_output = pid_angle_yaw.output_max;
    else if (yaw_output < pid_angle_yaw.output_min)
        yaw_output = pid_angle_yaw.output_min;
    
    /* 更新误差记录 */
    pid_angle_yaw.last_error = pid_angle_yaw.error;
    pid_angle_yaw.output = yaw_output;
    
    /* 根据控制模式选择输出策略 */
    switch (pid_control_state.mode)
    {
        case PID_MODE_STRAIGHT_DRIVE:
            /* 直行模式：基础速度 + 偏航修正 */
            motor_a_speed = pid_control_state.base_speed + yaw_output;
            motor_b_speed = pid_control_state.base_speed - yaw_output;
            break;
            
        case PID_MODE_TURN_IN_PLACE:
            /* 原地转向模式：纯差速，无基础速度 */
            motor_a_speed = yaw_output;
            motor_b_speed = -yaw_output;
            break;
            
        default:
            /* 其他模式或兼容旧代码：纯差速 */
            motor_a_speed = yaw_output;
            motor_b_speed = -yaw_output;
            break;
    }
    
    /* 设置电机目标速度，让底层PID根据正负自动确定方向 */
    PID_SetSpeed(PID_MOTOR_A, motor_a_speed);
    PID_SetSpeed(PID_MOTOR_B, motor_b_speed);
}

/**
 * @brief 简化的Yaw控制接口
 * @param target_yaw: 目标偏航角(度)
 * @retval None
 */
void YawControl_SetTarget(float target_yaw)
{
    Angle_PID_SetTarget(ANGLE_YAW, target_yaw);
}

/**
 * @brief 设置差速控制
 * @param base_speed: 基础速度(RPM)
 * @param yaw_correction: 偏航修正值(RPM)
 * @retval None
 */
void PID_SetDifferentialSpeed(float base_speed, float yaw_correction)
{
    float motor_a_speed, motor_b_speed;
    
    /* 计算差速：基础速度 ± 偏航修正 */
    motor_a_speed = base_speed + yaw_correction;  // 右轮
    motor_b_speed = base_speed - yaw_correction;  // 左轮
    
    /* 设置电机目标速度 */
    PID_SetSpeed(PID_MOTOR_A, motor_a_speed);
    PID_SetSpeed(PID_MOTOR_B, motor_b_speed);
    
    /* 更新控制状态 */
    pid_control_state.mode = PID_MODE_DIFFERENTIAL;
    pid_control_state.base_speed = base_speed;
}

/**
 * @brief 设置直行驱动模式（保持偏航角）
 * @param target_speed: 目标直行速度(RPM)
 * @param target_yaw: 目标偏航角(度)
 * @retval None
 */
void PID_SetStraightDrive(float target_speed, float target_yaw)
{
    /* 更新控制状态 */
    pid_control_state.mode = PID_MODE_STRAIGHT_DRIVE;
    pid_control_state.base_speed = target_speed;
    pid_control_state.target_yaw = target_yaw;
    pid_control_state.angle_control_active = 1;
    
    /* 设置角度PID目标值 */
    Angle_PID_SetTarget(ANGLE_YAW, target_yaw);
    
    /* 注意：实际的差速控制会在 Angle_PID_Update 中进行 */
    /* 这里不直接设置电机速度，而是依赖角度PID的输出 */
}

/**
 * @brief 设置原地转向模式
 * @param target_yaw: 目标偏航角(度)
 * @retval None
 */
void PID_SetTurnInPlace(float target_yaw)
{
    /* 更新控制状态 */
    pid_control_state.mode = PID_MODE_TURN_IN_PLACE;
    pid_control_state.base_speed = 0.0f;  // 原地转向，基础速度为0
    pid_control_state.target_yaw = target_yaw;
    pid_control_state.angle_control_active = 1;
    
    /* 设置角度PID目标值 */
    Angle_PID_SetTarget(ANGLE_YAW, target_yaw);
    
    /* 原地转向完全依赖角度PID的差速输出 */
}

/**
 * @brief 停止所有电机
 * @retval None
 */
void PID_StopAll(void)
{
    /* 停止电机 */
    PID_SetSpeed(PID_MOTOR_A, 0.0f);
    PID_SetSpeed(PID_MOTOR_B, 0.0f);
    
    /* 重置控制状态 */
    pid_control_state.mode = PID_MODE_MANUAL;
    pid_control_state.base_speed = 0.0f;
    pid_control_state.angle_control_active = 0;
    
    /* 重置角度PID */
    Angle_PID_Reset(ANGLE_YAW);
}