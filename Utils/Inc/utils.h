/**
 * @file utils.h
 * @brief 工具函数
 */

#ifndef __UTILS_H
#define __UTILS_H

#include "main.h"

/**
 * @brief 微秒延时函数
 * @param us 延时微秒数
 */
void HAL_DelayMicroseconds(uint32_t us);
void DWT_Init(void);

#endif /* __UTILS_H */