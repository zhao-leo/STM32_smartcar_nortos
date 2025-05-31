/**
 * @file PS2Mouse.c
 * @brief PS/2鼠标驱动实现
 */

#include "PS2Mouse.h"
#include "utils.h"
#include <stdio.h>

// 私有变量
static GPIO_TypeDef *PS2_CLOCK_PORT;
static uint16_t PS2_CLOCK_PIN;
static GPIO_TypeDef *PS2_DATA_PORT;
static uint16_t PS2_DATA_PIN;
static uint8_t PS2_MODE;
static uint8_t PS2_INITIALIZED = 0;
static uint8_t PS2_ENABLED = 0;
static uint8_t PS2_DISABLED = 1;

// 私有函数声明
static void PS2Mouse_PullHigh(GPIO_TypeDef *port, uint16_t pin);
static void PS2Mouse_PullLow(GPIO_TypeDef *port, uint16_t pin);
static void PS2Mouse_Write(uint8_t data);
static uint8_t PS2Mouse_ReadByte(void);
static uint8_t PS2Mouse_ReadBit(void);
static int16_t PS2Mouse_ReadMovementX(uint8_t status);
static int16_t PS2Mouse_ReadMovementY(uint8_t status);
static void PS2Mouse_SetMode(uint8_t mode);

// 初始化PS/2鼠标
void PS2Mouse_Init(GPIO_TypeDef *clockPort, uint16_t clockPin,
                   GPIO_TypeDef *dataPort, uint16_t dataPin, uint8_t mode)
{

    printf("PS2Mouse: 开始初始化... 模式=%d\r\n", mode);
    // 保存引脚信息
    PS2_CLOCK_PORT = clockPort;
    PS2_CLOCK_PIN = clockPin;
    PS2_DATA_PORT = dataPort;
    PS2_DATA_PIN = dataPin;
    PS2_MODE = mode;

    // 设置为输入上拉模式
    PS2Mouse_PullHigh(PS2_CLOCK_PORT, PS2_CLOCK_PIN);
    PS2Mouse_PullHigh(PS2_DATA_PORT, PS2_DATA_PIN);

    HAL_Delay(20);

    printf("PS2Mouse: 发送重置命令...\r\n");
    // 发送重置命令
    PS2Mouse_Write(0xFF);
    uint8_t ack = PS2Mouse_ReadByte(); // 读取确认字节
    printf("PS2Mouse: 重置应答=0x%02X (预期:0xFA)\r\n", ack);

    HAL_Delay(20);

    PS2Mouse_ReadByte(); // 读取空白字节
    PS2Mouse_ReadByte(); // 读取空白字节

    HAL_Delay(20);

    // 设置工作模式
    if (PS2_MODE == PS2_REMOTE_MODE)
    {
        PS2Mouse_SetRemoteMode();
    }
    else
    {
        PS2Mouse_EnableDataReporting(); // 启用数据报告
    }

    HAL_DelayMicroseconds(100);

    PS2_INITIALIZED = 1;
}

// 读取鼠标数据
uint8_t PS2Mouse_ReadData(PS2Mouse_Data_t *mouseData)
{
    uint8_t status;

    PS2Mouse_Write(0xEB);
    PS2Mouse_ReadByte();
    
    // 读取状态字节和移动数据
    status = PS2Mouse_ReadByte();
    
    // 解析按钮状态
    mouseData->leftBtn = (status & 0x01) ? 1 : 0;
    mouseData->rightBtn = (status & 0x02) ? 1 : 0;
    mouseData->middleBtn = (status & 0x04) ? 1 : 0;

    // 读取X和Y移动
    mouseData->x = PS2Mouse_ReadMovementX(status);
    mouseData->y = PS2Mouse_ReadMovementY(status);

    return 1; // 成功读取
}

// 设置远程模式
void PS2Mouse_SetRemoteMode(void)
{
    PS2Mouse_SetMode(0xF0);
    PS2_MODE = PS2_REMOTE_MODE;
}

// 设置流模式
void PS2Mouse_SetStreamMode(void)
{
    PS2Mouse_SetMode(0xEA);
    PS2_MODE = PS2_STREAM_MODE;
}

// 启用数据报告
void PS2Mouse_EnableDataReporting(void)
{
    if (!PS2_ENABLED)
    {
        PS2Mouse_Write(0xF4);
        PS2Mouse_ReadByte(); // 读取确认字节
        PS2_ENABLED = 1;
        PS2_DISABLED = 0;
    }
}

// 禁用数据报告
void PS2Mouse_DisableDataReporting(void)
{
    if (!PS2_DISABLED)
    {
        PS2Mouse_Write(0xF5);
        PS2Mouse_ReadByte(); // 读取确认字节
        PS2_DISABLED = 1;
        PS2_ENABLED = 0;
    }
}

// 设置采样率
void PS2Mouse_SetSampleRate(uint8_t rate)
{
    if (PS2_MODE == PS2_STREAM_MODE)
    {
        PS2Mouse_DisableDataReporting();
    }

    PS2Mouse_Write(0xF3);
    PS2Mouse_ReadByte(); // 读取确认字节

    PS2Mouse_Write(rate);
    PS2Mouse_ReadByte(); // 读取确认字节

    if (PS2_MODE == PS2_STREAM_MODE)
    {
        PS2Mouse_EnableDataReporting();
    }

    HAL_DelayMicroseconds(100);
}

// 设置分辨率
void PS2Mouse_SetResolution(uint8_t resolution)
{
    if (PS2_MODE == PS2_STREAM_MODE)
    {
        PS2Mouse_DisableDataReporting();
    }

    PS2Mouse_Write(0xE8);
    PS2Mouse_ReadByte(); // 读取确认字节

    PS2Mouse_Write(resolution);
    PS2Mouse_ReadByte(); // 读取确认字节

    if (PS2_MODE == PS2_STREAM_MODE)
    {
        PS2Mouse_EnableDataReporting();
    }

    HAL_Delay(1);
}

// 设置缩放比例2:1
void PS2Mouse_SetScaling2To1(void)
{
    PS2Mouse_SetMode(0xE7);
}

// 设置缩放比例1:1
void PS2Mouse_SetScaling1To1(void)
{
    PS2Mouse_SetMode(0xE6);
}

// 设置模式（内部函数）
static void PS2Mouse_PullHigh(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 关键改变：设置为输入上拉模式，让上拉电阻自然拉高线路
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(port, &GPIO_InitStruct);
    // 不需要WritePin，输入模式下无效
}

static void PS2Mouse_PullLow(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 先设置为输出低，再初始化，确保没有毛刺
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // 使用开漏输出，更适合PS/2协议
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

// 写一个字节到PS/2设备
static void PS2Mouse_Write(uint8_t data)
{
    uint8_t parity = 1; // 奇校验位

    // 禁用中断，确保时序准确
    __disable_irq();

    // 准备发送
    PS2Mouse_PullHigh(PS2_DATA_PORT, PS2_DATA_PIN);
    PS2Mouse_PullHigh(PS2_CLOCK_PORT, PS2_CLOCK_PIN);
    HAL_DelayMicroseconds(300);

    // 主机接管时钟
    PS2Mouse_PullLow(PS2_CLOCK_PORT, PS2_CLOCK_PIN);
    HAL_DelayMicroseconds(300);

    // 拉低数据线表示开始传输
    PS2Mouse_PullLow(PS2_DATA_PORT, PS2_DATA_PIN);
    HAL_DelayMicroseconds(10);

    // 释放时钟线，鼠标将接管时钟
    PS2Mouse_PullHigh(PS2_CLOCK_PORT, PS2_CLOCK_PIN);

    // 等待鼠标拉低时钟线
    uint32_t timeout = HAL_GetTick() + 100; // 100ms超时
    while (HAL_GPIO_ReadPin(PS2_CLOCK_PORT, PS2_CLOCK_PIN) == 1)
    {
        if (HAL_GetTick() > timeout)
        {
            __enable_irq(); // 确保恢复中断
            return;       // 或其他错误处理
        }
    }

    // 发送8位数据
    for (int i = 0; i < 8; i++)
    {
        if (data & 0x01)
        {
            PS2Mouse_PullHigh(PS2_DATA_PORT, PS2_DATA_PIN);
        }
        else
        {
            PS2Mouse_PullLow(PS2_DATA_PORT, PS2_DATA_PIN);
        }

        // 等待时钟周期
        while (HAL_GPIO_ReadPin(PS2_CLOCK_PORT, PS2_CLOCK_PIN) == GPIO_PIN_RESET)
        {
        }
        while (HAL_GPIO_ReadPin(PS2_CLOCK_PORT, PS2_CLOCK_PIN) == GPIO_PIN_SET)
        {
        }

        parity ^= (data & 0x01); // 更新校验位
        data = data >> 1;        // 准备下一位
    }

    // 发送校验位
    if (parity)
    {
        PS2Mouse_PullHigh(PS2_DATA_PORT, PS2_DATA_PIN);
    }
    else
    {
        PS2Mouse_PullLow(PS2_DATA_PORT, PS2_DATA_PIN);
    }

    // 等待时钟周期
    while (HAL_GPIO_ReadPin(PS2_CLOCK_PORT, PS2_CLOCK_PIN) == GPIO_PIN_RESET)
    {
    }
    while (HAL_GPIO_ReadPin(PS2_CLOCK_PORT, PS2_CLOCK_PIN) == GPIO_PIN_SET)
    {
    }

    // 发送停止位
    PS2Mouse_PullHigh(PS2_DATA_PORT, PS2_DATA_PIN);
    HAL_DelayMicroseconds(50);

    // 等待鼠标确认
    while (HAL_GPIO_ReadPin(PS2_CLOCK_PORT, PS2_CLOCK_PIN) == GPIO_PIN_SET)
    {
    }
    while ((HAL_GPIO_ReadPin(PS2_CLOCK_PORT, PS2_CLOCK_PIN) == GPIO_PIN_RESET) ||
           (HAL_GPIO_ReadPin(PS2_DATA_PORT, PS2_DATA_PIN) == GPIO_PIN_RESET))
    {
    }

    // 主机接管时钟以阻止数据传输
    PS2Mouse_PullLow(PS2_CLOCK_PORT, PS2_CLOCK_PIN);

    // 重新启用中断
    __enable_irq();
}

// 读取一个字节
static uint8_t PS2Mouse_ReadByte(void)
{
    uint8_t data = 0;

    // 禁用中断，确保时序准确
    __disable_irq();

    // 释放时钟和数据线
    PS2Mouse_PullHigh(PS2_CLOCK_PORT, PS2_CLOCK_PIN);
    PS2Mouse_PullHigh(PS2_DATA_PORT, PS2_DATA_PIN);
    HAL_DelayMicroseconds(50);

    // 等待设备开始传输
    while (HAL_GPIO_ReadPin(PS2_CLOCK_PORT, PS2_CLOCK_PIN) == GPIO_PIN_SET)
    {
        // 超时处理可以在这里添加
    }

    HAL_DelayMicroseconds(5);

    // 等待开始位完成
    while (HAL_GPIO_ReadPin(PS2_CLOCK_PORT, PS2_CLOCK_PIN) == GPIO_PIN_RESET)
    {
    }

    // 接收8位数据
    for (int i = 0; i < 8; i++)
    {
        if (PS2Mouse_ReadBit())
        {
            data |= (1 << i);
        }
    }

    // 读取校验位（不处理）
    PS2Mouse_ReadBit();

    // 读取停止位（应该为1）
    PS2Mouse_ReadBit();

    // 主机接管时钟以阻止数据传输
    PS2Mouse_PullLow(PS2_CLOCK_PORT, PS2_CLOCK_PIN);

    // 重新启用中断
    __enable_irq();

    return data;
}

// 读取一位
static uint8_t PS2Mouse_ReadBit(void)
{
    // 等待时钟下降沿
    while (HAL_GPIO_ReadPin(PS2_CLOCK_PORT, PS2_CLOCK_PIN) == GPIO_PIN_SET)
    {
    }

    // 读取数据位
    uint8_t bit = HAL_GPIO_ReadPin(PS2_DATA_PORT, PS2_DATA_PIN);

    // 等待时钟上升沿
    while (HAL_GPIO_ReadPin(PS2_CLOCK_PORT, PS2_CLOCK_PIN) == GPIO_PIN_RESET)
    {
    }

    return bit;
}

// 解析X轴移动数据
static int16_t PS2Mouse_ReadMovementX(uint8_t status)
{
    int16_t x = PS2Mouse_ReadByte();

    // 如果是负数，扩展符号位
    if (status & 0x10)
    {
        x |= 0xFF00; // 符号扩展
    }

    return x;
}

// 解析Y轴移动数据
static int16_t PS2Mouse_ReadMovementY(uint8_t status)
{
    int16_t y = PS2Mouse_ReadByte();

    // 如果是负数，扩展符号位
    if (status & 0x20)
    {
        y |= 0xFF00; // 符号扩展
    }

    return y;
}

static void PS2Mouse_SetMode(uint8_t mode)
{
    if (PS2_MODE == PS2_STREAM_MODE)
    {
        PS2Mouse_DisableDataReporting();
    }

    PS2Mouse_Write(mode);
    PS2Mouse_ReadByte(); // 读取确认字节

    if (PS2_MODE == PS2_STREAM_MODE && mode != 0xF5) // 0xF5是禁用数据报告的命令
    {
        PS2Mouse_EnableDataReporting();
    }

    HAL_Delay(10);
}