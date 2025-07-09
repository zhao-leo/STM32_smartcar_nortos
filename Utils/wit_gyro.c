/**
 ******************************************************************************
 * @file    wit_gyro.c
 * @brief   封装wit_sdk的JY901S的驱动程序
 * @author  Mingyuan Zhao
 * @version V1.0.0
 * @date    2025-07-09
 ******************************************************************************
 * @attention
 *
 * 本文件实现了WIT运动传感器(JY901S/JY901B)的驱动程序，基于STM32 HAL库。
 * 提供传感器初始化、数据读取和控制功能，通过I2C接口与传感器通信。
 *
 * 主要功能：
 * - 自动检测传感器I2C地址
 * - 读取加速度、陀螺仪、角度和磁力计数据
 * - 基于HAL库的I2C通信
 *
 * 硬件要求：
 * - 支持I2C外设的STM32微控制器
 * - WIT运动传感器(JY901S/JY901B或兼容型号)
 * - I2C上拉电阻(通常为4.7kΩ)
 *
 * 引脚连接：
 * - I2C1_SCL -> 传感器SCL
 * - I2C1_SDA -> 传感器SDA
 * - VCC -> 3.3V或5V
 * - GND -> 地线
 *
 * 使用示例：
 * @code
 * // 初始化驱动
 * if(WIT_Driver_Init() == WIT_HAL_OK) {
 *     while(1) {
 *         WIT_Driver_ReadData();
 *         if(WIT_Driver_IsDataReady()) {
 *             WIT_Data_t* data = WIT_Driver_GetData();
 *             // 使用传感器数据
 *             WIT_Driver_ClearDataFlag();
 *         }
 *         HAL_Delay(100);
 *     }
 * }
 * @endcode
 *
 ******************************************************************************
 */

#include "wit_gyro.h"

// HAL库I2C读写函数实现
static int32_t HAL_IICwriteBytes(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    if(HAL_I2C_Mem_Write(&hi2c1, ucAddr << 1, ucReg, I2C_MEMADD_SIZE_8BIT, p_ucVal, uiLen, 1000) == HAL_OK)
        return WIT_HAL_OK;
    return WIT_HAL_ERROR;
}

static int32_t HAL_IICreadBytes(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    if(HAL_I2C_Mem_Read(&hi2c1, ucAddr << 1, ucReg, I2C_MEMADD_SIZE_8BIT, p_ucVal, uiLen, 1000) == HAL_OK)
        return WIT_HAL_OK;
    return WIT_HAL_ERROR;
}

// 内部变量（从main.c移入）
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
static WIT_Data_t wit_data = {0};
static uint8_t device_found = 0;
static uint8_t device_addr = 0x50;

// 内部函数声明
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static void AutoScanSensor(void);
static void ShowHelp(void);
static void CmdProcess(void);

// 驱动初始化函数
int32_t WIT_Driver_Init(void)
{
    // 注册HAL库I2C读写函数
    WitI2cFuncRegister(HAL_IICwriteBytes, HAL_IICreadBytes);
    WitRegisterCallBack(CopeSensorData);
    WitDelayMsRegister(Delayms);
    
    printf("\r\n********************** WIT-Motion IIC Driver ************************\r\n");
    
    // 自动扫描设备
    AutoScanSensor();
    
    if(device_found)
    {
        printf("WIT sensor initialized successfully at address: 0x%02X\r\n", device_addr);
        return WIT_HAL_OK;
    }
    else
    {
        printf("WIT sensor initialization failed\r\n");
        return WIT_HAL_ERROR;
    }
}

// 读取数据
int32_t WIT_Driver_ReadData(void)
{
    if(!device_found) return WIT_HAL_ERROR;
    
    WitReadReg(AX, 12);
    CmdProcess(); // 处理命令
    
    if(s_cDataUpdate)
    {
        // 更新数据结构
        for(int i = 0; i < 3; i++)
        {
            wit_data.acc[i] = sReg[AX+i] / 32768.0f * 16.0f;
            wit_data.gyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
            wit_data.angle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
        }
        wit_data.mag[0] = sReg[HX];
        wit_data.mag[1] = sReg[HY];
        wit_data.mag[2] = sReg[HZ];
        
        wit_data.data_ready = 1;
        return WIT_HAL_OK;
    }
    return WIT_HAL_BUSY;
}

// 获取数据指针
WIT_Data_t* WIT_Driver_GetData(void)
{
    return &wit_data;
}

// 检查数据是否就绪
uint8_t WIT_Driver_IsDataReady(void)
{
    return wit_data.data_ready;
}

// 清除数据就绪标志
void WIT_Driver_ClearDataFlag(void)
{
    wit_data.data_ready = 0;
    s_cDataUpdate = 0;
}

// 处理外部命令
void WIT_Driver_ProcessCommand(char cmd)
{
    s_cCmd = cmd;
}

// ===== 以下是从main.c移过来的内部函数 =====

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
    for(int i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ: s_cDataUpdate |= ACC_UPDATE; break;
            case GZ: s_cDataUpdate |= GYRO_UPDATE; break;
            case HZ: s_cDataUpdate |= MAG_UPDATE; break;
            case Yaw: s_cDataUpdate |= ANGLE_UPDATE; break;
            default: s_cDataUpdate |= READ_UPDATE; break;
        }
        uiReg++;
    }
}

static void Delayms(uint16_t ucMs)
{
    HAL_Delay(ucMs);
}

static void AutoScanSensor(void)
{
    for(int i = 0; i < 0x7F; i++)
    {
        WitInit(WIT_PROTOCOL_I2C, i);
        int iRetry = 2;
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            HAL_Delay(5);
            if(s_cDataUpdate != 0)
            {
                printf("Found sensor at address: 0x%02X\r\n", i);
                device_addr = i;
                device_found = 1;
                ShowHelp();
                return;
            }
            iRetry--;
        }while(iRetry);
    }
    printf("No sensor found, please check connections\r\n");
}

static void ShowHelp(void)
{
    printf("\r\n************************ WIT_DRIVER_HELP ************************\r\n");
    printf("Available commands:\r\n");
    printf("  'a' - Start acceleration calibration\r\n");
    printf("  'm' - Start magnetic calibration\r\n");
    printf("  'e' - End magnetic calibration\r\n");
    printf("  'h' - Show this help\r\n");
    printf("******************************************************************\r\n");
}

static void CmdProcess(void)
{
    switch(s_cCmd)
    {
        case 'a':	
            if(WitStartAccCali() != WIT_HAL_OK) 
                printf("Acceleration calibration error\r\n");
            break;
        case 'm':	
            if(WitStartMagCali() != WIT_HAL_OK) 
                printf("Start magnetic calibration error\r\n");
            break;
        case 'e':	
            if(WitStopMagCali() != WIT_HAL_OK) 
                printf("End magnetic calibration error\r\n");
            break;
        case 'h':	
            ShowHelp();
            break;
        default: 
            return;
    }
    s_cCmd = 0xff;
}