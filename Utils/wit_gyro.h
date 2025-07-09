/**
 ******************************************************************************
 * @file    wit_gyro.h
 * @brief   WIT运动传感器驱动程序头文件
 * @author  Mingyuan Zhao
 * @version V1.0.0
 * @date    2025-07-09
 ******************************************************************************
 * @attention
 *
 * 本头文件定义了WIT运动传感器驱动的接口函数和数据结构。
 * 提供访问传感器数据的接口，包括加速度、陀螺仪、角度和磁力计测量值。
 *
 * 支持的传感器：
 * - JY901S (9轴传感器)
 * - JY901B (9轴传感器)
 * - 其他WIT兼容传感器
 *
 * 通信协议：
 * - I2C接口 (默认地址: 0x50)
 * - 自动检测传感器地址 (0x50-0x57)
 * - 基于HAL库实现
 *
 ******************************************************************************
 */

#ifndef __WIT_DRIVER_H
#define __WIT_DRIVER_H

#include <stdint.h>
#include <stdio.h>
#include "wit_c_sdk.h"
#include "REG.h"
#include "main.h"
#include "i2c.h"

// 数据结构定义
typedef struct {
    float acc[3];     // 加速度 X,Y,Z
    float gyro[3];    // 角速度 X,Y,Z  
    float angle[3];   // 角度 Roll,Pitch,Yaw
    int16_t mag[3];   // 磁力计 X,Y,Z
    uint8_t data_ready; // 数据就绪标志
} WIT_Data_t;

// 驱动接口函数
int32_t WIT_Driver_Init(void);
int32_t WIT_Driver_ReadData(void);
WIT_Data_t* WIT_Driver_GetData(void);
void WIT_Driver_ProcessCommand(char cmd);
uint8_t WIT_Driver_IsDataReady(void);
void WIT_Driver_ClearDataFlag(void);


#endif