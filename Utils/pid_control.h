#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "encoder.h"
#include "motor.h"

/* PID控制器结构体 */
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    
    float setpoint;     // 设定值（目标值）
    float feedback;     // 反馈值（当前值）
    float error;        // 当前误差
    float last_error;   // 上次误差
    float prev_error;   // 上上次误差
    float integral;     // 积分项
    float derivative;   // 微分项
    
    float output;       // PID输出
    float output_min;   // 输出下限
    float output_max;   // 输出上限
    
    uint8_t direction;  // 电机方向
} PID_TypeDef;

/* PID控制器参数（可调整） */
// 为电机A和B分别定义PID参数
#define PID_KP_A         0.9f     // 电机A比例系数默认值
#define PID_KI_A         0.45f     // 电机A积分系数默认值
#define PID_KD_A         0.6f     // 电机A微分系数默认值

#define PID_KP_B         2.0f     // 电机B比例系数默认值
#define PID_KI_B         1.4f     // 电机B积分系数默认值
#define PID_KD_B         1.0f     // 电机B微分系数默认值
#define PID_OUTPUT_MAX  999.0f   // 输出最大值（对应PWM最大值）
#define PID_OUTPUT_MIN  0.0f     // 输出最小值
#define PID_SAMPLE_TIME 100.0f   // PID采样时间(ms)，应与编码器采样时间一致

/* 电机PID控制器枚举 */
typedef enum {
    PID_MOTOR_A = 1,
    PID_MOTOR_B = 2
} PID_Motor_ID;

/* 函数声明 */
void PID_Init(PID_TypeDef *pid);
float PID_Compute(PID_TypeDef *pid);
void PID_SetSpeed(PID_Motor_ID motor_id, float target_rpm);
void PID_Update(void);

/* 外部变量声明 */
extern PID_TypeDef pid_motor_a;
extern PID_TypeDef pid_motor_b;
// 声明各自的全局调节参数
extern float pid_kp_a;
extern float pid_ki_a;
extern float pid_kd_a;

extern float pid_kp_b;
extern float pid_ki_b;
extern float pid_kd_b;

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROL_H */