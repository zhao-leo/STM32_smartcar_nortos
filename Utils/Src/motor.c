#include "../Inc/motor.h"
#include "tim.h"

#define PWM_PERIOD 7199  // TIM1的周期值

void Motor_Init(void)
{
  // 启动PWM输出
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void Motor_SetDutyCycle(uint8_t dutyCycle)
{
  // 限制占空比在0-100之间
  if (dutyCycle > 100)
    dutyCycle = 100;
    
  // 计算PWM值并设置
  uint32_t pwmValue = (PWM_PERIOD + 1) * dutyCycle / 100;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmValue);
}

void Motor_Control(MotorState_t state)
{
  switch (state) {
    case MOTOR_STOP:
      Motor_SetDutyCycle(0);
      break;
    case MOTOR_FORWARD:
      // 根据需要设置方向引脚（如果有的话）
      // HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
      break;
    case MOTOR_REVERSE:
      // 根据需要设置方向引脚（如果有的话）
      // HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
      break;
  }
}