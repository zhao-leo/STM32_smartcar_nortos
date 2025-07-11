# 角度环PID控制系统

## 概述

本项目为STM32F103平台实现了一个双层PID控制系统，包含：
- **内环（速度环）**：控制电机转速，使用编码器反馈
- **外环（角度环）**：控制机器人姿态角度，使用陀螺仪反馈

## 系统架构

```
陀螺仪(JY61P) → 角度环PID → 速度设定值 → 速度环PID → PWM输出 → 电机
                    ↑                           ↑
                目标角度                    编码器反馈
```

## 文件结构

### 核心文件
- `Utils/pid_control.h/c` - PID控制核心实现（速度环 + 角度环）
- `Utils/angle_control.h/c` - 角度控制高级封装接口
- `Utils/angle_control_examples.c` - 使用示例和说明

### 相关文件
- `Utils/encoder.h/c` - 编码器处理
- `Utils/motor.h/c` - 电机控制
- `Utils/jy61p.h/c` - 陀螺仪数据处理

## 功能特性

### 1. 双层PID控制
- **角度环**：外环控制，输出目标转速
- **速度环**：内环控制，输出PWM值
- **级联控制**：角度环输出作为速度环输入

### 2. 多轴角度控制
- **Roll轴**：横滚角控制（平衡）
- **Pitch轴**：俯仰角控制  
- **Yaw轴**：偏航角控制（转向）

### 3. 控制模式
- `ANGLE_CONTROL_DISABLED` - 禁用角度控制
- `ANGLE_CONTROL_BALANCE` - 平衡模式（主要使用Roll轴）
- `ANGLE_CONTROL_TURN` - 转向模式（主要使用Yaw轴）
- `ANGLE_CONTROL_FULL` - 全轴控制模式

### 4. 高级功能
- 角度环绕处理（处理-180°到180°的跳变）
- 动态参数调整
- 灵敏度控制
- 积分饱和限制
- 紧急停止和重置

## 使用方法

### 1. 系统初始化
```c
// 在main函数中已自动添加
AngleControl_Init();
```

### 2. 基本平衡控制
```c
// 启用平衡模式，目标Roll角度为0度
AngleControl_SetBalance(0.0f);
```

### 3. 转向控制
```c
// 转向到90度方向
AngleControl_SetTurn(90.0f);
```

### 4. 全轴控制
```c
// 设置目标角度：Roll=0°, Pitch=0°, Yaw=180°
AngleControl_SetTarget(0.0f, 0.0f, 180.0f);
```

### 5. 参数调整
```c
// 调整控制灵敏度
AngleControl_SetSensitivity(1.2f, 1.0f); // (平衡灵敏度, 转向灵敏度)
```

## PID参数配置

### 角度环参数（在pid_control.h中）
```c
#define ANGLE_PID_KP 5.0f       // 比例系数
#define ANGLE_PID_KI 0.2f       // 积分系数  
#define ANGLE_PID_KD 2.0f       // 微分系数
#define ANGLE_PID_OUTPUT_MAX 200.0f  // 输出最大值(RPM)
#define ANGLE_PID_OUTPUT_MIN -200.0f // 输出最小值(RPM)
```

### 速度环参数（在pid_control.h中）
```c
#define PID_KP_A 0.9f   // 电机A比例系数
#define PID_KI_A 0.45f  // 电机A积分系数
#define PID_KD_A 0.6f   // 电机A微分系数

#define PID_KP_B 2.0f   // 电机B比例系数
#define PID_KI_B 1.4f   // 电机B积分系数
#define PID_KD_B 1.0f   // 电机B微分系数
```

## 时序控制

### 定时器配置
- **定时器频率**：1ms（1kHz）
- **角度环更新**：每20ms（50Hz）
- **速度环更新**：每20ms（50Hz，与角度环同步）
- **传感器读取**：每100ms

### 中断处理
系统在`HAL_TIM_PeriodElapsedCallback`中自动更新：
```c
if (angle_pid_sample >= 19) // 每20ms
{
    Angle_PID_Update();  // 更新角度环
    PID_Update();        // 更新速度环
    angle_pid_sample = 0;
}
```

## 调试和优化

### 1. PID参数调试
建议调试顺序：
1. 先调试速度环PID参数
2. 再调试角度环PID参数
3. 最后优化两者的协调

### 2. 监控变量
可以通过串口输出监控以下变量：
- `Roll`, `Pitch`, `Yaw` - 当前角度
- `pid_angle_roll.error` - 角度误差
- `pid_angle_roll.output` - 角度环输出
- `encoderA.speed_rpm`, `encoderB.speed_rpm` - 电机转速

### 3. 常见问题
- **振荡**：降低比例系数(Kp)
- **响应慢**：提高比例系数(Kp)或微分系数(Kd)
- **稳态误差**：适当增加积分系数(Ki)
- **积分饱和**：检查积分限幅设置

## API参考

### 主要函数
- `AngleControl_Init()` - 初始化角度控制系统
- `AngleControl_Enable()` - 启用角度控制
- `AngleControl_Disable()` - 禁用角度控制
- `AngleControl_SetTarget()` - 设置目标角度
- `AngleControl_SetBalance()` - 设置平衡目标
- `AngleControl_SetTurn()` - 设置转向目标
- `AngleControl_Reset()` - 重置系统

### 底层函数
- `Angle_PID_Init()` - 初始化角度PID
- `Angle_PID_Update()` - 更新角度PID
- `PID_SetSpeed()` - 设置电机目标转速
- `PID_Update()` - 更新速度PID

## 扩展功能

### 1. 添加新的控制模式
可以在`angle_control.h`中添加新的`AngleControlMode_TypeDef`枚举值。

### 2. 多机器人配置
系统支持扩展到多个电机的机器人平台。

### 3. 传感器融合
可以集成更多传感器（如加速度计、磁力计）来提高控制精度。

## 注意事项

1. **安全性**：确保在调试时设置合适的输出限幅
2. **稳定性**：建议从小的PID参数开始逐步调整
3. **频率匹配**：确保角度环频率高于或等于速度环频率
4. **传感器校准**：确保陀螺仪和编码器都正确校准

## 更新日志

- 2025-07-11：初始版本，实现双层PID控制系统
- 功能：角度环PID + 速度环PID + 高级封装接口
