/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "string.h"
#include "stdlib.h"
#include "pid_control.h"
#define UART_RX_BUFFER_SIZE 64
static uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
static uint8_t uartRxData;
static uint8_t rxBufferIndex = 0;
static uint8_t rxBufferFlag = 0;
/* USER CODE END 0 */

UART_HandleTypeDef huart3;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include <stdio.h>

// 重定向printf到USART3
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}



// 初始化串口接�??
void UART_StartReceive(void)
{
  HAL_UART_Receive_IT(&huart3, &uartRxData, 1);
}

// 串口接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
    // 如果接收到回车或换行，表示一条命令接收完�?
    if(uartRxData == '\r' || uartRxData == '\n')
    {
      if(rxBufferIndex > 0)
      {
        // 添加结束�?
        uartRxBuffer[rxBufferIndex] = 0;
        // 设置接收完成标志
        rxBufferFlag = 1;
        // 重置索引
        rxBufferIndex = 0;
      }
    }
    // 否则将接收到的数据存入缓冲区
    else if(rxBufferIndex < UART_RX_BUFFER_SIZE - 1)
    {
      uartRxBuffer[rxBufferIndex++] = uartRxData;
    }
    
    // 继续接收下一个字�?
    HAL_UART_Receive_IT(&huart3, &uartRxData, 1);
  }
}

/**
  * @brief 解析串口接收到的PID参数
  * @retval �?
  */
void UART_ParsePIDCommand(void)
{
  if(rxBufferFlag)
  {
    // 清除接收标志
    rxBufferFlag = 0;
    
    char *cmd = (char*)uartRxBuffer;
    char param;
    float value;
    
    // 调试输出接收到的原始命令
    printf("接收到命�?: %s\r\n", cmd);
    
    // 解析命令格式: p=1.23 �? i=0.45 �? d=0.67
    if(sscanf(cmd, "%c=%f", &param, &value) == 2)
    {
      // 根据参数类型更新对应的PID�?
      switch(param)
      {
        case 'p':
        case 'P':
          pid_kp_a = value;
          pid_motor_a.integral = 0.0f;
          pid_motor_a.derivative = 0.0f;
          printf("set A Kp=%.2f\r\n", pid_kp_a);
          break;
          
        case 'i':
        case 'I':
          pid_ki_a = value;
          pid_motor_a.integral = 0.0f;
          pid_motor_a.derivative = 0.0f;
          printf("set A Ki=%.2f\r\n", pid_ki_a);
          break;
          
        case 'd':
        case 'D':
          pid_kd_a = value;
          pid_motor_a.integral = 0.0f;
          pid_motor_a.derivative = 0.0f;
          printf("set A Kd=%.2f\r\n", pid_kd_a);
          break;
          
        case 'q':
        case 'Q':
          pid_kp_b = value;
          pid_motor_b.integral = 0.0f;
          pid_motor_b.derivative = 0.0f;
          printf("set B Kp=%.2f\r\n", pid_kp_b);
          break;
          
        case 'w':
        case 'W':
          pid_ki_b = value;
          pid_motor_b.integral = 0.0f;
          pid_motor_b.derivative = 0.0f;
          printf("set B Ki=%.2f\r\n", pid_ki_b);
          break;
          
        case 'e':
        case 'E':
          pid_kd_b = value;
          pid_motor_b.integral = 0.0f;
          pid_motor_b.derivative = 0.0f;
          printf("set B Kd=%.2f\r\n", pid_kd_b);
          break;
          
        default:
          printf("未知参数: %c\r\n", param);
          break;
      }
    }
    else
    {
      // 尝试其他解析方式，检查缓冲区是否包含有效的等�?
      char *equalsign = strchr(cmd, '=');
      if (equalsign != NULL) {
        // 如果找到等号，可能是格式解析问题
        param = cmd[0]; // 获取第一个字符作为参�?
        value = atof(equalsign + 1); // 从等号后面开始解析浮点数
        
        printf("备用解析: 参数=%c, �?=%.2f\r\n", param, value);
        
        // 使用备用解析结果处理参数
        switch(param)
        {
          case 'p':
          case 'P':
            pid_kp_a = value;
            pid_motor_a.integral = 0.0f;
            pid_motor_a.derivative = 0.0f;
            printf("set A Kp=%.2f\r\n", pid_kp_a);
            break;
            
          case 'i':
          case 'I':
            pid_ki_a = value;
            pid_motor_a.integral = 0.0f;
            pid_motor_a.derivative = 0.0f;
            printf("set A Ki=%.2f\r\n", pid_ki_a);
            break;
            
          case 'd':
          case 'D':
            pid_kd_a = value;
            pid_motor_a.integral = 0.0f;
            pid_motor_a.derivative = 0.0f;
            printf("set A Kd=%.2f\r\n", pid_kd_a);
            break;
            
          case 'q':
          case 'Q':
            pid_kp_b = value;
            pid_motor_b.integral = 0.0f;
            pid_motor_b.derivative = 0.0f;
            printf("set B Kp=%.2f\r\n", pid_kp_b);
            break;
            
          case 'w':
          case 'W':
            pid_ki_b = value;
            pid_motor_b.integral = 0.0f;
            pid_motor_b.derivative = 0.0f;
            printf("set B Ki=%.2f\r\n", pid_ki_b);
            break;
            
          case 'e':
          case 'E':
            pid_kd_b = value;
            pid_motor_b.integral = 0.0f;
            pid_motor_b.derivative = 0.0f;
            printf("set B Kd=%.2f\r\n", pid_kd_b);
            break;
            
          default:
            printf("未知参数: %c\r\n", param);
            break;
        }
      } else {
        printf("please put p=1.23 or i=0.45 or d=0.67\r\n");
      }
    }
  }
}
/* USER CODE END 1 */
