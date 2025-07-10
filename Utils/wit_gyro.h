/**
 ******************************************************************************
 * @file    wit_gyro.h
 * @brief   WIT运动传感器驱动程序头文件
 * @author  Mingyuan Zhao
 * @version V1.0.1
 * @date    2025-07-10
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
 * - 串口通信 (UART)
 * - 支持波特率自动检测
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
#include "usart.h"

// 波特率定义
#define WIT_BAUD_4800    1
#define WIT_BAUD_9600    2
#define WIT_BAUD_19200   3
#define WIT_BAUD_38400   4
#define WIT_BAUD_57600   5
#define WIT_BAUD_115200  6
#define WIT_BAUD_230400  7
#define WIT_BAUD_460800  8
#define WIT_BAUD_921600  9

// 数据更新标志定义
#define WIT_ACC_UPDATE    0x01
#define WIT_GYRO_UPDATE   0x02
#define WIT_ANGLE_UPDATE  0x04
#define WIT_MAG_UPDATE    0x08
#define WIT_ALL_UPDATE    0x0F

// 数据结构定义
typedef struct {
    float acc[3];     // 加速度 X,Y,Z
    float gyro[3];    // 角速度 X,Y,Z  
    float angle[3];   // 角度 Roll,Pitch,Yaw
    int16_t mag[3];   // 磁力计 X,Y,Z
    uint8_t data_ready; // 数据就绪标志
    uint8_t acc_updated;  // 加速度数据已更新标志
    uint8_t gyro_updated; // 陀螺仪数据已更新标志
    uint8_t angle_updated; // 角度数据已更新标志
    uint8_t mag_updated;  // 磁力计数据已更新标志
} WIT_Data_t;

// 驱动接口函数
int32_t WIT_Driver_Init(uint32_t baudRate);
int32_t WIT_Driver_ReadData(void);
WIT_Data_t* WIT_Driver_GetData(void);
void WIT_Driver_ProcessCommand(char cmd);
uint8_t WIT_Driver_IsDataReady(void);
void WIT_Driver_ClearDataFlag(void);
void WIT_Driver_RegisterUartConfig(void (*configFunc)(uint32_t));
void WIT_Driver_ClearUpdateFlag(uint8_t flag_type);

// 配置函数
int32_t WIT_Driver_SetBandwidth(uint8_t bandwidth);
int32_t WIT_Driver_SetBaudRate(uint8_t baudRate);
int32_t WIT_Driver_SetOutputRate(uint8_t rate);
int32_t WIT_Driver_SetContent(uint8_t content);

#endif