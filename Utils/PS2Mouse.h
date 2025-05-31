/**
 * @file PS2Mouse.h
 * @brief PS/2鼠标驱动头文件
 */

#ifndef __PS2MOUSE_H
#define __PS2MOUSE_H

#include "main.h"
#include <stdint.h>

// 操作模式定义
#define PS2_REMOTE_MODE 1
#define PS2_STREAM_MODE 2

// 鼠标数据结构
typedef struct {
    uint8_t leftBtn;     // 左键状态
    uint8_t middleBtn;   // 中键状态
    uint8_t rightBtn;    // 右键状态
    int16_t x;           // X轴移动
    int16_t y;           // Y轴移动
} PS2Mouse_Data_t;

// 初始化PS/2鼠标
void PS2Mouse_Init(GPIO_TypeDef* clockPort, uint16_t clockPin, 
                   GPIO_TypeDef* dataPort, uint16_t dataPin, uint8_t mode);

// 读取鼠标数据
uint8_t PS2Mouse_ReadData(PS2Mouse_Data_t* mouseData);

// 设置远程模式
void PS2Mouse_SetRemoteMode(void);

// 设置流模式
void PS2Mouse_SetStreamMode(void);

// 启用数据报告（流模式）
void PS2Mouse_EnableDataReporting(void);

// 禁用数据报告（流模式）
void PS2Mouse_DisableDataReporting(void);

// 设置采样率
void PS2Mouse_SetSampleRate(uint8_t rate);

// 设置分辨率
void PS2Mouse_SetResolution(uint8_t resolution);

// 设置缩放比例2:1
void PS2Mouse_SetScaling2To1(void);

// 设置缩放比例1:1
void PS2Mouse_SetScaling1To1(void);

#endif /* __PS2MOUSE_H */