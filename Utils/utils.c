/**
 * @file utils.c
 * @brief 工具函数实现
 */

#include "utils.h"

/**
 * @brief 微秒延时函数
 * @note 此函数适用于72MHz系统时钟
 * @param us 延时微秒数
 */
void HAL_DelayMicroseconds(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    
    while ((DWT->CYCCNT - start) < cycles);
}

/**
 * @brief 初始化DWT计数器（在main函数中调用一次）
 */
void DWT_Init(void)
{
    // 启用DWT功能
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    
    // DWT复位计数器
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    
    // 启用DWT计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}