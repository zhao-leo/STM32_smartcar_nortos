#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

/* 电机方向控制宏定义 */
#define MOTOR_FORWARD     0
#define MOTOR_BACKWARD    1
#define MOTOR_STOP        2

/* 函数声明 */
void Motor_Init(void);
void Motor_SetSpeed(uint8_t motorID, uint16_t speed, uint8_t direction);
void Motor_A_SetPWM(uint16_t pwm);
void Motor_B_SetPWM(uint16_t pwm);
void Motor_Stop(uint8_t motorID);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */