#include "motor.h"

/**
 * @brief 电机初始化函数
 * @note 初始化PWM和方向控制引脚
 * @retval None
 */
void Motor_Init(void)
{
    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // 启动电机A的PWM
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  // 启动电机B的PWM
    
    /* 初始状态下停止电机 */
    Motor_Stop(1);  // 停止电机A
    Motor_Stop(2);  // 停止电机B
}

/**
 * @brief 设置电机A的PWM值
 * @param pwm: PWM值(0-999，对应0-100%占空比)
 * @retval None
 */
void Motor_A_SetPWM(uint16_t pwm)
{
    /* 限制PWM值在有效范围内 */
    if(pwm > 999)
        pwm = 999;
    
    /* 设置PWM值 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);
}

/**
 * @brief 设置电机B的PWM值
 * @param pwm: PWM值(0-999，对应0-100%占空比)
 * @retval None
 */
void Motor_B_SetPWM(uint16_t pwm)
{
    /* 限制PWM值在有效范围内 */
    if(pwm > 999)
        pwm = 999;
    
    /* 设置PWM值 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm);
}

/**
 * @brief 设置电机速度和方向
 * @param motorID: 电机ID，1表示电机A，2表示电机B
 * @param speed: PWM值(0-999，对应0-100%占空比)
 * @param direction: 方向，0表示正转，1表示反转，2表示停止
 * @retval None
 */
void Motor_SetSpeed(uint8_t motorID, uint16_t speed, uint8_t direction)
{
    /* 限制速度值在有效范围内 */
    if(speed > 999)
        speed = 999;
    
    if(motorID == 1)  // 电机A
    {
        switch(direction)
        {
            case MOTOR_FORWARD:
                HAL_GPIO_WritePin(GPIOC, MotorA_control_1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOC, MotorA_control_2_Pin, GPIO_PIN_RESET);
                Motor_A_SetPWM(speed);
                break;
            
            case MOTOR_BACKWARD:
                HAL_GPIO_WritePin(GPIOC, MotorA_control_1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, MotorA_control_2_Pin, GPIO_PIN_SET);
                Motor_A_SetPWM(speed);
                break;
            
            case MOTOR_STOP:
                HAL_GPIO_WritePin(GPIOC, MotorA_control_1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, MotorA_control_2_Pin, GPIO_PIN_RESET);
                Motor_A_SetPWM(0);
                break;
            
            default:
                break;
        }
    }
    else if(motorID == 2)  // 电机B
    {
        switch(direction)
        {
            case MOTOR_FORWARD:
                HAL_GPIO_WritePin(GPIOA, MotorB_control_1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, MotorB_control_2_Pin, GPIO_PIN_RESET);
                Motor_B_SetPWM(speed);
                break;
            
            case MOTOR_BACKWARD:
                HAL_GPIO_WritePin(GPIOA, MotorB_control_1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, MotorB_control_2_Pin, GPIO_PIN_SET);
                Motor_B_SetPWM(speed);
                break;
            
            case MOTOR_STOP:
                HAL_GPIO_WritePin(GPIOA, MotorB_control_1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, MotorB_control_2_Pin, GPIO_PIN_RESET);
                Motor_B_SetPWM(0);
                break;
            
            default:
                break;
        }
    }
}

/**
 * @brief 停止电机
 * @param motorID: 电机ID，1表示电机A，2表示电机B
 * @retval None
 */
void Motor_Stop(uint8_t motorID)
{
    if(motorID == 1)  // 电机A
    {
        HAL_GPIO_WritePin(GPIOC, MotorA_control_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, MotorA_control_2_Pin, GPIO_PIN_RESET);
        Motor_A_SetPWM(0);
    }
    else if(motorID == 2)  // 电机B
    {
        HAL_GPIO_WritePin(GPIOA, MotorB_control_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, MotorB_control_2_Pin, GPIO_PIN_RESET);
        Motor_B_SetPWM(0);
    }
}