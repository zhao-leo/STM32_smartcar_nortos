#ifndef __ANGLE_CONTROL_H
#define __ANGLE_CONTROL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "pid_control.h"

/* 角度控制模式枚举 */
typedef enum
{
    ANGLE_CONTROL_DISABLED = 0,  // 角度控制关闭
    ANGLE_CONTROL_BALANCE = 1,   // 平衡模式（主要使用Roll轴）
    ANGLE_CONTROL_TURN = 2,      // 转向模式（主要使用Yaw轴）
    ANGLE_CONTROL_FULL = 3       // 全轴控制模式
} AngleControlMode_TypeDef;

/* 角度控制结构体 */
typedef struct
{
    AngleControlMode_TypeDef mode;     // 控制模式
    uint8_t enabled;                   // 是否启用角度控制
    float target_roll;                 // 目标Roll角度
    float target_pitch;                // 目标Pitch角度  
    float target_yaw;                  // 目标Yaw角度
    float balance_sensitivity;         // 平衡灵敏度 (0.1 - 2.0)
    float turn_sensitivity;            // 转向灵敏度 (0.1 - 2.0)
} AngleControl_TypeDef;

/* 函数声明 */
void AngleControl_Init(void);
void AngleControl_Enable(void);
void AngleControl_Disable(void);
void AngleControl_SetMode(AngleControlMode_TypeDef mode);
void AngleControl_SetTarget(float roll, float pitch, float yaw);
void AngleControl_SetBalance(float target_roll);
void AngleControl_SetTurn(float target_yaw);
void AngleControl_SetSensitivity(float balance_sens, float turn_sens);
void AngleControl_Reset(void);
AngleControl_TypeDef* AngleControl_GetStatus(void);

/* 外部变量声明 */
extern AngleControl_TypeDef angle_control;

#ifdef __cplusplus
}
#endif

#endif /* __ANGLE_CONTROL_H */
