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
#include "PS2Mouse.h"
#include "utils.h"
#include <string.h>
#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include "pid_control.h"
#include "icm42688_hal.h"
#include "ahrs_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ICM42688_CS_PORT GPIOA
#define ICM42688_CS_PIN GPIO_PIN_3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// SPI通信函数实现
void your_spi_init(void)
{
  // SPI已经在MX_SPI1_Init()中初始化了，这里配置CS引脚
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = ICM42688_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ICM42688_CS_PORT, &GPIO_InitStruct);

  // CS引脚默认拉高
  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);
}

uint8_t your_spi_read(uint8_t reg)
{
  uint8_t tx_data[2] = {reg | 0x80, 0x00}; // 读操作：�?高位�?1，加dummy字节
  uint8_t rx_data[2] = {0};

  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET); // CS拉低
  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, HAL_MAX_DELAY);  // 同时发�?�和接收2字节
  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);   // CS拉高

  return rx_data[1]; // 返回第二个字节的数据
}

void your_spi_write(uint8_t reg, uint8_t value)
{
  uint8_t tx_data[2];
  tx_data[0] = reg & 0x7F; // 写操作：�?????高位�?????0
  tx_data[1] = value;

  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET); // CS拉低
  HAL_SPI_Transmit(&hspi1, tx_data, 2, HAL_MAX_DELAY);                  // 发�?�寄存器地址和数�?????
  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);   // CS拉高
}

void your_spi_read_burst(uint8_t reg, uint8_t *data, uint8_t len)
{
  uint8_t tx_data = reg | 0x80; // 读操作：�?高位�?1

  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET); // CS拉低
  
  // 先发送地�?
  HAL_SPI_Transmit(&hspi1, &tx_data, 1, HAL_MAX_DELAY);
  
  // 再连续读取数�?
  HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
  
  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);   // CS拉高
}

// 添加基础SPI测试函数
void test_spi_basic(void)
{
  uint8_t tx_data = 0xAA;  // 测试数据
  uint8_t rx_data = 0x00;
  
  printf("发�?�测试数�?: 0x%02X\r\n", tx_data);
  
  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, 1000);
  HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);
  
  printf("SPI: %d, receve: 0x%02X\r\n", status, rx_data);
  
  // 如果状�?�不是HAL_OK，说明SPI通信有问�?
  if(status != HAL_OK) {
    printf("SPI error !\r\n");
  }
}

void my_delay_ms(uint16_t ms)
{
  HAL_Delay((uint32_t)ms);
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

MPU6050_t MPU6050;

PS2Mouse_Data_t mouseData;

// 显示用变�?????????????
int16_t displayValue_x = 0;
int16_t displayValue_y = 0;
int16_t times = 0;
int16_t sampleing = 0;

uint8_t serialBuf[100];

extern Encoder_TypeDef encoderA;
extern Encoder_TypeDef encoderB;
ahrs_euler_angle_t attitudex;
// 实现通信接口
icm42688_comm_t icm42688_comm = {
    .init = your_spi_init,
    .read_reg = your_spi_read,
    .write_reg = your_spi_write,
    .read_regs = your_spi_read_burst};

// 实现系统接口
icm42688_system_t icm42688_system = {
    .delay_ms = my_delay_ms};

// 配置传感器参�?????
icm42688_config_t icm42688_config = {
    .interface_type = ICM42688_INTERFACE_SPI,
    .acc_sample = ICM42688_ACC_SAMPLE_SGN_16G,
    .gyro_sample = ICM42688_GYRO_SAMPLE_SGN_2000DPS,
    .sample_rate = ICM42688_SAMPLE_RATE_1000};

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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
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
  // while (MPU6050_Init(&hi2c1) == 1)
  //   ;
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
  your_spi_init();
  printf("=== SPI状�?�检�? ===\r\n");
  
  // �?查SPI是否正确初始�?
  if(hspi1.State == HAL_SPI_STATE_READY) {
    printf("ready\r\n");
  } else {
    printf("error (%d)\r\n", hspi1.State);
  }

    // �?查SPI寄存器配�?
  printf("SPI1 CR1: 0x%04X\r\n", hspi1.Instance->CR1);
  printf("SPI1 CR2: 0x%04X\r\n", hspi1.Instance->CR2);
  printf("SPI1 SR: 0x%04X\r\n", hspi1.Instance->SR);
  
  // 测试基本的SPI发�?�接�?
  printf("=== 基础SPI测试 ===\r\n");
  test_spi_basic();
  

  HAL_TIM_Base_Start_IT(&htim2);
  // HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
  // HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_2);
  // HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  // HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  // Motor_Init();
  // Encoder_Init();
  // PID_Init(&pid_motor_a);
  // PID_SetSpeed(PID_MOTOR_A, 120);
  // PID_Init(&pid_motor_b);
  HAL_Delay(100);
  // 初始化传感器和姿态解�?????
  icm42688_hal_init(&icm42688_comm, &icm42688_system, &icm42688_config);
  printf("begin ICM42688...\r\n");

  // 先测试SPI通信

    // 多次尝试读取设备ID
  for(int i = 0; i < 5; i++) {
    uint8_t test_id = your_spi_read(ICM42688_WHO_AM_I);
    printf("�?%d次读取ID: 0x%02X (期望: 0x47)\r\n", i+1, test_id);
    HAL_Delay(10);
  }

  uint8_t device_config = your_spi_read(0x11);
  printf("DEVICE_CONFIG: 0x%02X\r\n", device_config);

  // 再进行初始化
  uint8_t init_result = icm42688_init();
  printf("init result: %d\r\n", init_result);
  if (icm42688_init() != 0)
  {
    printf("ICM42688 fail,code: %d\r\n", init_result);
    if (init_result == 1)
    {
      printf("1\r\n");
    }
    else if (init_result == 2)
    {
      printf("2\r\n");
    }
  }
  ahrs_init();
  ahrs_init_attitude_offset();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    icm42688_get_acc();
    icm42688_get_gyro();
    printf("ACC Raw: X=%d, Y=%d, Z=%d\r\n", icm42688_acc.x, icm42688_acc.y, icm42688_acc.z);

    // 打印�?????螺仪原始数据
    printf("GYRO Raw: X=%d, Y=%d, Z=%d\r\n", icm42688_gyro.x, icm42688_gyro.y, icm42688_gyro.z);
    // UART_ParsePIDCommand();

    // ssd1306_Fill(Black);
    // 读取鼠标数据
    // if (PS2Mouse_ReadData(&mouseData))
    // {
    //   // 更新显示�?????????????
    //   displayValue_x += mouseData.x;
    //   displayValue_y += mouseData.y;

    //   // 限制范围�?????????????0-100
    //   if (displayValue_x > 9999)
    //     displayValue_x = 0;
    //   if (displayValue_x < -9999)
    //     displayValue_x = 0;
    //   if (displayValue_y > 9999)
    //     displayValue_y = 0;
    //   if (displayValue_y < -9999)
    //     displayValue_y = 0;
    // 在OLED上显示鼠标数�?????????????

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
    HAL_ResumeTick();
    /*this is a test code to read MPU6050 data

    MPU6050_Read_All(&hi2c1, &MPU6050);
    char x_str[16] = {0};
    char y_str[16] = {0};
    char temp_str[16] = {0};
    sprintf(temp_str, "%d", 3);
    sprintf(x_str, "%.2f", MPU6050.KalmanAngleX);
    sprintf(y_str, "%.2f", MPU6050.KalmanAngleY);

    printf("MPUx: %s, MPUy: %s\r\n", x_str, y_str);
    */
    //  if(times==99){
    //   Encoder_Update(&encoderA, ENCODER_A);
    //   Encoder_Update(&encoderB, ENCODER_B);
    //   PID_Update();
    //   printf("EncoderA: %.2f, EncoderB: %.2f, Kp:%.2f, Ki:%.2f, Kd:%.2f, i:%.2f, d:%.2f\r\n", encoderA.speed_rpm, encoderB.speed_rpm, pid_motor_a.Kp, pid_motor_a.Ki, pid_motor_a.Kd, pid_motor_a.integral,pid_motor_a.derivative);
    //   times=0;
    //  }
    //  else {
    //   times++;
    //  }
    if (times == 9)
    {
      ahrs_get_attitude(&attitudex);
      printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", attitudex.roll, attitudex.pitch, attitudex.yaw);
      times = 0;
    }
    else
    {
      times++;
    }
    HAL_SuspendTick();
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
