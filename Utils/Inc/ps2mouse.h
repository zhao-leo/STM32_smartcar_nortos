#ifndef PS2MOUSE_H
#define PS2MOUSE_H

#include "main.h"
#include "cmsis_os2.h"

// 定义PS2鼠标模式
#define STREAM 0
#define REMOTE 1

// PS2鼠标数据结构
typedef struct {
  int status;    // 状态字节
  int x;         // X方向移动
  int y;         // Y方向移动
  int leftBtn;   // 左键状态
  int rightBtn;  // 右键状态
  int middleBtn; // 中键状态
} PS2MouseData_t;

// PS2鼠标初始化
void PS2Mouse_Init(GPIO_TypeDef* clockPort, uint16_t clockPin, 
                  GPIO_TypeDef* dataPort, uint16_t dataPin, int mode);

// 设置远程模式
void PS2Mouse_SetRemoteMode(void);

// 设置流模式
void PS2Mouse_SetStreamMode(void);

// 设置采样率
void PS2Mouse_SetSampleRate(int rate);

// 设置分辨率
void PS2Mouse_SetResolution(int resolution);

// 启用数据报告
void PS2Mouse_EnableDataReporting(void);

// 禁用数据报告
void PS2Mouse_DisableDataReporting(void);

// 读取鼠标数据
void PS2Mouse_ReadData(PS2MouseData_t* mouseData);

#endif /* PS2MOUSE_H */