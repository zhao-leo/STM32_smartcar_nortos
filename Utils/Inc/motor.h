// 创建文件: motor_control.h
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdint.h>

// 电机控制状态枚举
typedef enum {
  MOTOR_STOP,
  MOTOR_FORWARD,
  MOTOR_REVERSE
} MotorState_t;

// 电机控制初始化
void Motor_Init(void);

// 设置电机占空比
void Motor_SetDutyCycle(uint8_t dutyCycle);

// 控制电机状态
void Motor_Control(MotorState_t state);

#endif /* MOTOR_CONTROL_H */