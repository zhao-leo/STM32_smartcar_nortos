#include "ps2mouse.h"
#include "main.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"

// 鼠标配置变量
static GPIO_TypeDef* CLOCK_PORT;
static uint16_t CLOCK_PIN;
static GPIO_TypeDef* DATA_PORT;
static uint16_t DATA_PIN;
static int _mode;
static int _initialized = 0;
static int _enabled = 0;
static int _disabled = 1;

// FreeRTOS相关
static TaskHandle_t xMouseTaskHandle = NULL;
#define PS2_MOUSE_TIMEOUT_MS 50  // 超时时间，单位ms

// 私有函数声明
static void PS2Mouse_Write(uint8_t data);
static uint8_t PS2Mouse_ReadByte(void);
static uint8_t PS2Mouse_ReadBit(void);
static void PS2Mouse_PullLow(GPIO_TypeDef* port, uint16_t pin);
static void PS2Mouse_PullHigh(GPIO_TypeDef* port, uint16_t pin);
static void PS2Mouse_SetMode(int data);
static uint8_t PS2Mouse_Read(void);
static int PS2Mouse_ReadMovementX(int status);
static int PS2Mouse_ReadMovementY(int status);
static uint8_t PS2Mouse_HasEvenParity(uint8_t x);
static uint8_t PS2Mouse_WaitForCLK(uint8_t bit);
static uint8_t PS2Mouse_IsCLKset(void);
static uint8_t PS2Mouse_IsDATAset(void);

// PS2鼠标初始化
void PS2Mouse_Init(GPIO_TypeDef* clockPort, uint16_t clockPin, 
                 GPIO_TypeDef* dataPort, uint16_t dataPin, int mode) {
  // 获取当前任务句柄，用于后续超时处理
  xMouseTaskHandle = xTaskGetCurrentTaskHandle();
  
  // 保存引脚配置
  CLOCK_PORT = clockPort;
  CLOCK_PIN = clockPin;
  DATA_PORT = dataPort;
  DATA_PIN = dataPin;
  _mode = mode;
  
  // 初始化引脚状态
  PS2Mouse_PullHigh(CLOCK_PORT, CLOCK_PIN);
  PS2Mouse_PullHigh(DATA_PORT, DATA_PIN);
  
  // 等待鼠标准备好 - 使用vTaskDelay而不是osDelay
  vTaskDelay(pdMS_TO_TICKS(20));
  
  // 发送复位命令
  PS2Mouse_Write(0xFF);
  
  // 读取确认字节
  PS2Mouse_ReadByte();
  
  // 延时
  vTaskDelay(pdMS_TO_TICKS(20));
  
  // 读取额外数据（复位后的状态信息）
  PS2Mouse_ReadByte();
  PS2Mouse_ReadByte();
  
  vTaskDelay(pdMS_TO_TICKS(20));
  
  // 根据模式设置
  if (_mode == REMOTE) {
    PS2Mouse_SetRemoteMode();
  } else {
    PS2Mouse_EnableDataReporting();
  }
  
  _initialized = 1;
}

// 设置鼠标模式
static void PS2Mouse_SetMode(int data) {
  if (_mode == STREAM) {
    PS2Mouse_DisableDataReporting();
  }
  
  PS2Mouse_Write(data);
  PS2Mouse_ReadByte();
  
  if (_mode == STREAM) {
    PS2Mouse_EnableDataReporting();
  }
}

// 设置远程模式
void PS2Mouse_SetRemoteMode(void) {
  PS2Mouse_SetMode(0xF0);
  _mode = REMOTE;
}

// 设置流模式
void PS2Mouse_SetStreamMode(void) {
  PS2Mouse_SetMode(0xEA);
  _mode = STREAM;
}

// 设置采样率
void PS2Mouse_SetSampleRate(int rate) {
  if (_mode == STREAM) {
    PS2Mouse_DisableDataReporting();
  }
  
  PS2Mouse_Write(0xF3);
  PS2Mouse_ReadByte();
  PS2Mouse_Write(rate);
  PS2Mouse_ReadByte();
  
  if (_mode == STREAM) {
    PS2Mouse_EnableDataReporting();
  }
}

// 设置2:1缩放
void PS2Mouse_SetScaling21(void) {
  PS2Mouse_SetMode(0xE7);
}

// 设置1:1缩放
void PS2Mouse_SetScaling11(void) {
  PS2Mouse_SetMode(0xE6);
}

// 启用数据报告（仅在流模式下有效）
void PS2Mouse_EnableDataReporting(void) {
  if (!_enabled) {
    PS2Mouse_Write(0xF4);
    PS2Mouse_ReadByte();
    _enabled = 1;
    _disabled = 0;
  }
}

// 禁用数据报告
void PS2Mouse_DisableDataReporting(void) {
  if (!_disabled) {
    PS2Mouse_Write(0xF5);
    PS2Mouse_ReadByte();
    _disabled = 1;
    _enabled = 0;
  }
}

// 设置分辨率
void PS2Mouse_SetResolution(int resolution) {
  if (_mode == STREAM) {
    PS2Mouse_DisableDataReporting();
  }
  
  PS2Mouse_Write(0xE8);
  PS2Mouse_ReadByte();
  PS2Mouse_Write(resolution);
  PS2Mouse_ReadByte();
  
  if (_mode == STREAM) {
    PS2Mouse_EnableDataReporting();
  }
}

// 检查偶校验位
static uint8_t PS2Mouse_HasEvenParity(uint8_t x) {
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return (~x) & 1;
}

// 检查时钟线状态
static uint8_t PS2Mouse_IsCLKset(void) {
  return (HAL_GPIO_ReadPin(CLOCK_PORT, CLOCK_PIN) == GPIO_PIN_SET);
}

// 检查数据线状态
static uint8_t PS2Mouse_IsDATAset(void) {
  return (HAL_GPIO_ReadPin(DATA_PORT, DATA_PIN) == GPIO_PIN_SET);
}

// 等待时钟线状态变化（带超时）
static uint8_t PS2Mouse_WaitForCLK(uint8_t bit) {
  TickType_t xStartTime = xTaskGetTickCount();
  
  while (PS2Mouse_IsCLKset() == bit) {
    // 检查是否超时 - 使用FreeRTOS的tick计数
    if ((xTaskGetTickCount() - xStartTime) > pdMS_TO_TICKS(PS2_MOUSE_TIMEOUT_MS)) {
      return 0; // 超时
    }
    // 短暂让出CPU - 重要的FreeRTOS优化
    taskYIELD();
  }
  return 1; // 成功
}

// 向PS2鼠标写入数据（实时性要求高）
static void PS2Mouse_Write(uint8_t data) {
  uint8_t i;
  uint8_t parity = 1;
  
  // 临时禁用任务切换，确保写操作不被中断
  taskENTER_CRITICAL();
  
  PS2Mouse_PullHigh(DATA_PORT, DATA_PIN);
  PS2Mouse_PullHigh(CLOCK_PORT, CLOCK_PIN);
  
  // 使用硬件延时而不是任务延时
  for(i=0; i<100; i++) {
    __NOP();
  }
  
  PS2Mouse_PullLow(CLOCK_PORT, CLOCK_PIN);
  
  for(i=0; i<100; i++) {
    __NOP();
  }
  
  PS2Mouse_PullLow(DATA_PORT, DATA_PIN);
  
  __NOP();
  __NOP();
  __NOP();
  
  PS2Mouse_PullHigh(CLOCK_PORT, CLOCK_PIN); // 开始位
  
  // 等待鼠标接管时钟
  uint8_t timeout = 200;
  while (PS2Mouse_IsCLKset() && timeout > 0) {
    timeout--;
    __NOP();
  }
  
  if (timeout == 0) {
    // 超时处理
    taskEXIT_CRITICAL();
    return;
  }
  
  // 时钟为低，可以发送数据
  for (i=0; i<8; i++) {
    if (data & 0x01) {
      PS2Mouse_PullHigh(DATA_PORT, DATA_PIN);
    } else {
      PS2Mouse_PullLow(DATA_PORT, DATA_PIN);
    }
    
    // 等待时钟周期 - 使用超时机制
    timeout = 100;
    while (!PS2Mouse_IsCLKset() && timeout > 0) {
      timeout--;
      __NOP();
    }
    if (timeout == 0) {
      taskEXIT_CRITICAL();
      return;
    }
    
    timeout = 100;
    while (PS2Mouse_IsCLKset() && timeout > 0) {
      timeout--;
      __NOP();
    }
    if (timeout == 0) {
      taskEXIT_CRITICAL();
      return;
    }
    
    parity = parity ^ (data & 0x01);
    data = data >> 1;
  }
  
  // 发送奇偶校验位
  if (parity) {
    PS2Mouse_PullHigh(DATA_PORT, DATA_PIN);
  } else {
    PS2Mouse_PullLow(DATA_PORT, DATA_PIN);
  }
  
  timeout = 100;
  while (!PS2Mouse_IsCLKset() && timeout > 0) {
    timeout--;
    __NOP();
  }
  if (timeout == 0) {
    taskEXIT_CRITICAL();
    return;
  }
  
  timeout = 100;
  while (PS2Mouse_IsCLKset() && timeout > 0) {
    timeout--;
    __NOP();
  }
  if (timeout == 0) {
    taskEXIT_CRITICAL();
    return;
  }
  
  PS2Mouse_PullHigh(DATA_PORT, DATA_PIN);
  
  // 延时
  for(i=0; i<20; i++) {
    __NOP();
  }
  
  timeout = 100;
  while (PS2Mouse_IsCLKset() && timeout > 0) {
    timeout--;
    __NOP();
  }
  if (timeout == 0) {
    taskEXIT_CRITICAL();
    return;
  }
  
  // 等待鼠标切换模式
  timeout = 200;
  while ((!PS2Mouse_IsCLKset() || !PS2Mouse_IsDATAset()) && timeout > 0) {
    timeout--;
    __NOP();
  }
  
  PS2Mouse_PullLow(CLOCK_PORT, CLOCK_PIN); // 锁定输入数据
  
  // 重新允许任务切换
  taskEXIT_CRITICAL();
}

// 读取鼠标数据
void PS2Mouse_ReadData(PS2MouseData_t* mouseData) {
  int data[3];
  
  if (_mode == REMOTE) {
    // 进入临界区保护读取过程
    taskENTER_CRITICAL();
    
    PS2Mouse_Write(0xEB); // 发送读取数据命令
    PS2Mouse_ReadByte();  // 读取确认字节
    
    data[0] = PS2Mouse_Read();                 // 状态字节
    data[1] = PS2Mouse_ReadMovementX(data[0]); // X移动数据
    data[2] = PS2Mouse_ReadMovementY(data[0]); // Y移动数据
    
    // 离开临界区
    taskEXIT_CRITICAL();
    
    // 更新数据结构
    mouseData->status = data[0];
    mouseData->x = data[1];
    mouseData->y = data[2];
    mouseData->leftBtn = data[0] & 0x01;
    mouseData->rightBtn = (data[0] & 0x02) >> 1;
    mouseData->middleBtn = (data[0] & 0x04) >> 2;
  }
}

// 读取单个字节
static uint8_t PS2Mouse_Read(void) {
  return PS2Mouse_ReadByte();
}

// 读取字节
static uint8_t PS2Mouse_ReadByte(void) {
  uint8_t data = 0;
  uint8_t i;
  uint8_t timeout;
  
  PS2Mouse_PullHigh(CLOCK_PORT, CLOCK_PIN);
  PS2Mouse_PullHigh(DATA_PORT, DATA_PIN);
  
  // 短暂延时
  for(i=0; i<20; i++) {
    __NOP();
  }
  
  // 等待时钟变低
  timeout = 100;
  while (PS2Mouse_IsCLKset() && timeout > 0) {
    timeout--;
    __NOP();
  }
  if (timeout == 0) return 0;
  
  // 延时
  __NOP();
  
  // 等待时钟变高（开始位）
  timeout = 100;
  while (!PS2Mouse_IsCLKset() && timeout > 0) {
    timeout--;
    __NOP();
  }
  if (timeout == 0) return 0;
  
  // 读取8位数据
  for (i=0; i<8; i++) {
    // 使用读取位函数（内部有超时控制）
    if (PS2Mouse_ReadBit()) {
      data |= (1 << i);
    }
  }
  
  PS2Mouse_ReadBit(); // 奇偶校验位
  PS2Mouse_ReadBit(); // 停止位应为1
  
  PS2Mouse_PullLow(CLOCK_PORT, CLOCK_PIN);
  
  return data;
}

// 读取单个位
static uint8_t PS2Mouse_ReadBit(void) {
  uint8_t bit;
  uint8_t timeout = 100;
  
  // 等待时钟变低
  while (PS2Mouse_IsCLKset() && timeout > 0) {
    timeout--;
    __NOP();
  }
  if (timeout == 0) return 0;
  
  // 读取数据位
  bit = PS2Mouse_IsDATAset();
  
  // 等待时钟变高
  timeout = 100;
  while (!PS2Mouse_IsCLKset() && timeout > 0) {
    timeout--;
    __NOP();
  }
  
  return bit;
}

// 读取X移动数据
static int PS2Mouse_ReadMovementX(int status) {
  int x = PS2Mouse_Read();
  
  if (status & 0x10) {
    // 处理负值（符号扩展）
    for(int i=8; i<16; i++) {
      x |= (1 << i);
    }
  }
  
  return x;
}

// 读取Y移动数据
static int PS2Mouse_ReadMovementY(int status) {
  int y = PS2Mouse_Read();
  
  if (status & 0x20) {
    // 处理负值（符号扩展）
    for(int i=8; i<16; i++) {
      y |= (1 << i);
    }
  }
  
  return y;
}

// 将引脚拉低
static void PS2Mouse_PullLow(GPIO_TypeDef* port, uint16_t pin) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 提高速度等级
  
  HAL_GPIO_Init(port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

// 将引脚拉高
static void PS2Mouse_PullHigh(GPIO_TypeDef* port, uint16_t pin) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 提高速度等级
  
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}