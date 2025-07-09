/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "utils.h"
#include <string.h>
#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include "pid_control.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

MPU6050_t MPU6050;


// 显示用变�??????????????????
int16_t displayValue_x = 0;
int16_t displayValue_y = 0;
int16_t times = 0;
int16_t sampleing = 0;
float Roll, Pitch, Yaw;

volatile uint8_t update_attitude_flag = 0;
uint8_t serialBuf[100];

MPU6050_t MPU6050={0};


extern Encoder_TypeDef encoderA;
extern Encoder_TypeDef encoderB;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  // Init uart receive
  UART_StartReceive();
  // DWT_Init();
  // // A0=CLOCK A1=DATA
  // PS2Mouse_Init(GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1, PS2_STREAM_MODE);
  // printf("PS/2鼠标初始化完成\r\n");
  // PS2Mouse_SetScaling2To1();
  // printf("系统已启动，串口配置完成！\r\n");
  // HAL_Delay(500);
  // while (MPU6050_Init(&hi2c1) == 1);
  // ssd1306_Init();
  // ssd1306_Fill(White);
  // ssd1306_UpdateScreen();
  // HAL_Delay(500);
  // ssd1306_Fill(Black);
  // ssd1306_UpdateScreen();
  // 显示标题
  // ssd1306_SetCursor(0, 0);
  // ssd1306_WriteString("Mouse Data:", Font_7x10, White);
  // ssd1306_UpdateScreen();

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  Motor_Init();
  Encoder_Init();
  Motor_SetSpeed(1, 200, MOTOR_FORWARD);
  Motor_SetSpeed(2, 200, MOTOR_FORWARD);
  // PID_Init(&pid_motor_a);
  // PID_Init(&pid_motor_b);

  // PID_SetSpeed(PID_MOTOR_A, 80);
  // PID_SetSpeed(PID_MOTOR_B, 60);
  // HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (update_attitude_flag == 199)
    {

    //   // printf("Roll: %.2f, Pitch: %.2f, Yaw:%.2f,", MPU6050.KalmanAngleX, MPU6050.KalmanAngleY,MPU6050.YawAngle);
    //   // printf("%.2f,", MPU6050.YawAngle);
    //   // printf("%.2f,%.2f,", displacementCalculator.displacementX, displacementCalculator.displacementY);
      update_attitude_flag =0;
      Encoder_Update(&encoderA, ENCODER_A);
      Encoder_Update(&encoderB, ENCODER_B);
    //   PID_Update();
      printf("A: %.2f, B:%.2f \r\n", encoderA.speed_rpm, encoderB.speed_rpm);
    //   MPU6050_Read_All(&hi2c1, &MPU6050);
    //   printf("%.2f\r\n", MPU6050.YawAngle);
    }
    if(sampleing==49){

      sampleing = 0;
    }

    // UART_ParsePIDCommand();

    // ssd1306_Fill(Black);
    // 读取鼠标数据
    // if (PS2Mouse_ReadData(&mouseData))
    // {
    //   // 更新显示�??????????????????
    //   displayValue_x += mouseData.x;
    //   displayValue_y += mouseData.y;

    //   // 限制范围�??????????????????0-100
    //   if (displayValue_x > 9999)
    //     displayValue_x = 0;
    //   if (displayValue_x < -9999)
    //     displayValue_x = 0;
    //   if (displayValue_y > 9999)
    //     displayValue_y = 0;
    //   if (displayValue_y < -9999)
    //     displayValue_y = 0;
    // 在OLED上显示鼠标数�??????????????????

    // char x_str[16]= {0};
    // char y_str[16]= {0};
    // sprintf(x_str, "%d", displayValue_x);
    // sprintf(y_str, "%d", displayValue_y);

    // char dx_str[16] = {0};
    // char dy_str[16] = {0};
    // sprintf(dx_str, "%d", mouseData.x);
    // sprintf(dy_str, "%d", mouseData.y);
    // if (mouseData.x != 0 || mouseData.y != 0)
    // {
    //   printf("x:");
    //   printf(x_str);
    //   printf("\r\n");
    //   printf("y:");
    //   printf(y_str);
    //   printf("\r\n");
    // }

    // ssd1306_SetCursor(0, 0);
    // ssd1306_WriteString("Mouse Data:", Font_7x10, White);
    // ssd1306_SetCursor(0, 10);
    // ssd1306_WriteString("X: ", Font_7x10, White);
    // ssd1306_SetCursor(20, 10);
    // ssd1306_WriteString(dx_str, Font_7x10, White);
    // ssd1306_SetCursor(0, 20);
    // ssd1306_WriteString("Y: ", Font_7x10, White);
    // ssd1306_SetCursor(20, 20);
    // ssd1306_WriteString(dy_str, Font_7x10, White);
    // ssd1306_UpdateScreen();

    //   HAL_Delay(2); // 延时2ms,即光电模块回报率
    // }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check if timer has triggered and update attitude
  // This is zhongduan functuon, every 1ms proceed once
  if (htim == &htim2)
  {
    /*this is a test code to read MPU6050 data

    */
    //  if(times<99){
    //   times++;
    //  }
    if(update_attitude_flag<199){
    update_attitude_flag ++;
    }
    if(sampleing<49){
      sampleing++;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
