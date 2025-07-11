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

/* 定义角度环PID控制器实例 */
PID_TypeDef pid_angle_roll;   // Roll轴角度PID控制器
PID_TypeDef pid_angle_pitch;  // Pitch轴角度PID控制器
PID_TypeDef pid_angle_yaw;    // Yaw轴角度PID控制器

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
 * @brief 初始化角度环PID控制器
 * @retval None
 */
void Angle_PID_Init(void)
{
    /* 初始化Roll轴PID控制器 */
    pid_angle_roll.Kp = angle_pid_kp;
    pid_angle_roll.Ki = angle_pid_ki;
    pid_angle_roll.Kd = angle_pid_kd;
    pid_angle_roll.setpoint = 0.0f;
    pid_angle_roll.feedback = 0.0f;
    pid_angle_roll.error = 0.0f;
    pid_angle_roll.last_error = 0.0f;
    pid_angle_roll.prev_error = 0.0f;
    pid_angle_roll.integral = 0.0f;
    pid_angle_roll.derivative = 0.0f;
    pid_angle_roll.output = 0.0f;
    pid_angle_roll.output_min = ANGLE_PID_OUTPUT_MIN;
    pid_angle_roll.output_max = ANGLE_PID_OUTPUT_MAX;
    
    /* 初始化Pitch轴PID控制器 */
    pid_angle_pitch.Kp = angle_pid_kp;
    pid_angle_pitch.Ki = angle_pid_ki;
    pid_angle_pitch.Kd = angle_pid_kd;
    pid_angle_pitch.setpoint = 0.0f;
    pid_angle_pitch.feedback = 0.0f;
    pid_angle_pitch.error = 0.0f;
    pid_angle_pitch.last_error = 0.0f;
    pid_angle_pitch.prev_error = 0.0f;
    pid_angle_pitch.integral = 0.0f;
    pid_angle_pitch.derivative = 0.0f;
    pid_angle_pitch.output = 0.0f;
    pid_angle_pitch.output_min = ANGLE_PID_OUTPUT_MIN;
    pid_angle_pitch.output_max = ANGLE_PID_OUTPUT_MAX;
    
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
 * @brief 设置角度环目标值
 * @param axis: 角度轴 (ANGLE_ROLL, ANGLE_PITCH, ANGLE_YAW)
 * @param target_angle: 目标角度(度)
 * @retval None
 */
void Angle_PID_SetTarget(PID_Angle_Axis axis, float target_angle)
{
    switch(axis)
    {
        case ANGLE_ROLL:
            pid_angle_roll.setpoint = target_angle;
            break;
        case ANGLE_PITCH:
            pid_angle_pitch.setpoint = target_angle;
            break;
        case ANGLE_YAW:
            pid_angle_yaw.setpoint = target_angle;
            break;
        default:
            break;
    }
}

/**
 * @brief 重置角度环PID状态
 * @param axis: 角度轴 (ANGLE_ROLL, ANGLE_PITCH, ANGLE_YAW)
 * @retval None
 */
void Angle_PID_Reset(PID_Angle_Axis axis)
{
    PID_TypeDef *pid = NULL;
    
    switch(axis)
    {
        case ANGLE_ROLL:
            pid = &pid_angle_roll;
            break;
        case ANGLE_PITCH:
            pid = &pid_angle_pitch;
            break;
        case ANGLE_YAW:
            pid = &pid_angle_yaw;
            break;
        default:
            return;
    }
    
    if(pid != NULL)
    {
        pid->error = 0.0f;
        pid->last_error = 0.0f;
        pid->prev_error = 0.0f;
        pid->integral = 0.0f;
        pid->derivative = 0.0f;
        pid->output = 0.0f;
    }
}

/**
 * @brief 角度环PID计算（专门处理角度环绕问题）
 * @param pid: PID控制器结构体指针
 * @param current_angle: 当前角度值
 * @retval PID输出值
 */
static float Angle_PID_Compute(PID_TypeDef *pid, float current_angle)
{
    /* 更新反馈值 */
    pid->feedback = current_angle;
    
    /* 计算角度误差（处理环绕） */
    pid->error = Angle_Difference(pid->setpoint, pid->feedback);
    
    /* 计算积分项 */
    pid->integral += pid->error;
    
    /* 积分项限幅，防止积分饱和 */
    float integral_limit = pid->output_max / pid->Ki;
    if (pid->integral > integral_limit)
        pid->integral = integral_limit;
    else if (pid->integral < -integral_limit)
        pid->integral = -integral_limit;
    
    /* 计算微分项 */
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
    pid->last_error = pid->error;
    
    return pid->output;
}

/**
 * @brief 更新角度环PID控制器
 * @note 此函数应在定时器中定期调用，建议调用频率与陀螺仪读取频率一致
 * @retval None
 */
void Angle_PID_Update(void)
{
    float roll_output, pitch_output, yaw_output;
    
    /* 更新角度环PID参数 */
    pid_angle_roll.Kp = angle_pid_kp;
    pid_angle_roll.Ki = angle_pid_ki;
    pid_angle_roll.Kd = angle_pid_kd;
    
    pid_angle_pitch.Kp = angle_pid_kp;
    pid_angle_pitch.Ki = angle_pid_ki;
    pid_angle_pitch.Kd = angle_pid_kd;
    
    pid_angle_yaw.Kp = angle_pid_kp;
    pid_angle_yaw.Ki = angle_pid_ki;
    pid_angle_yaw.Kd = angle_pid_kd;
    
    /* 计算各轴的角度环PID输出 */
    roll_output = Angle_PID_Compute(&pid_angle_roll, Roll);
    pitch_output = Angle_PID_Compute(&pid_angle_pitch, Pitch);
    yaw_output = Angle_PID_Compute(&pid_angle_yaw, Yaw);
    
    /* 根据角度环输出控制电机速度 */
    /* 这里可以根据你的机器人结构调整控制逻辑 */
    /* 示例：使用Roll和Yaw控制两个电机实现平衡和转向 */
    
    /* Roll轴控制平衡，两个电机反向补偿 */
    float balance_compensation = roll_output;
    
    /* Yaw轴控制转向，两个电机同向差速 */
    float turn_compensation = yaw_output;
    
    /* 计算每个电机的目标速度 */
    float motor_a_target = balance_compensation + turn_compensation;
    float motor_b_target = -balance_compensation + turn_compensation;
    
    /* 通过速度环PID控制电机 */
    PID_SetSpeed(PID_MOTOR_A, motor_a_target);
    PID_SetSpeed(PID_MOTOR_B, motor_b_target);
}