#include "encoder.h"
#include "tim.h"

/* 编码器实例 */
Encoder_TypeDef encoderA;  // 对应TIM3
Encoder_TypeDef encoderB;  // 对应TIM1

/**
 * @brief  编码器初始化
 * @param  None
 * @retval None
 */
void Encoder_Init(void)
{
    // 清除计数器值
    Encoder_Clear(ENCODER_A);
    Encoder_Clear(ENCODER_B);
    
    // 清零编码器结构体
    encoderA.count = 0;
    encoderA.last_count = 0;
    encoderA.diff = 0;
    encoderA.speed_rpm = 0.0f;
    encoderA.speed_rps = 0.0f;
    encoderA.direction = 0;
    
    encoderB.count = 0;
    encoderB.last_count = 0;
    encoderB.diff = 0;
    encoderB.speed_rpm = 0.0f;
    encoderB.speed_rps = 0.0f;
    encoderB.direction = 0;
}

/**
 * @brief  获取编码器计数值
 * @param  id: 编码器ID
 * @retval 计数值
 */
int16_t Encoder_GetCounter(Encoder_ID id)
{
    if (id == ENCODER_A)
    {
        return (int16_t)(__HAL_TIM_GET_COUNTER(&htim3));
    }
    else
    {
        return (int16_t)(__HAL_TIM_GET_COUNTER(&htim1));
    }
}

/**
 * @brief  清除编码器计数值
 * @param  id: 编码器ID
 * @retval None
 */
void Encoder_Clear(Encoder_ID id)
{
    if (id == ENCODER_A)
    {
        __HAL_TIM_SET_COUNTER(&htim3, 0);
    }
    else
    {
        __HAL_TIM_SET_COUNTER(&htim1, 0);
    }
}

/**
 * @brief  更新编码器状态
 * @param  encoder: 编码器结构体指针
 * @param  id: 编码器ID
 * @retval None
 * @note   此函数应该在固定时间间隔内调用，例如100ms
 */
void Encoder_Update(Encoder_TypeDef *encoder, Encoder_ID id)
{
    // 保存上次计数值
    encoder->last_count = encoder->count;
    
    // 读取当前计数值
    encoder->count = Encoder_GetCounter(id);
    
    // 计算差值（考虑计数器溢出情况）
    if (encoder->count < encoder->last_count)
    {
        // 如果当前值小于上次值，说明可能发生了向上溢出
        if (encoder->last_count - encoder->count > ENCODER_TIM_PERIOD/2)
        {
            encoder->diff = encoder->count + (ENCODER_TIM_PERIOD - encoder->last_count);
        }
        else
        {
            encoder->diff = encoder->count - encoder->last_count;
        }
    }
    else
    {
        // 如果当前值大于上次值，说明可能发生了向下溢出
        if (encoder->count - encoder->last_count > ENCODER_TIM_PERIOD/2)
        {
            encoder->diff = encoder->count - (ENCODER_TIM_PERIOD + encoder->last_count);
        }
        else
        {
            encoder->diff = encoder->count - encoder->last_count;
        }
    }

    // 判断方向
    if (encoder->diff > 0)
    {
        encoder->direction = 0;  // 正转
    }
    else if (encoder->diff < 0)
    {
        encoder->direction = 1;  // 反转
        encoder->diff = -encoder->diff;  // 取绝对值用于计算速度
    }
    
    // 计算转速 (counts/sample_time => RPM)
    // RPM = (diff * 60 * 1000) / (ENCODER_RESOLUTION * GEAR_RATIO * SAMPLE_TIME_MS)
    encoder->speed_rpm = (float)encoder->diff * 60.0f * 1000.0f / 
                         (ENCODER_RESOLUTION * GEAR_RATIO * SAMPLE_TIME_MS);
    
    // 计算RPS (RPM / 60)
    encoder->speed_rps = encoder->speed_rpm / 60.0f;
}