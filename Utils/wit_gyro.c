/**
 ******************************************************************************
 * @file    wit_gyro.c
 * @brief   封装wit_sdk的JY901S的驱动程序
 * @author  Mingyuan Zhao
 * @version V1.0.1
 * @date    2025-07-10
 ******************************************************************************
 * @attention
 *
 * 本文件实现了WIT运动传感器(JY901S/JY901B)的驱动程序，基于STM32 HAL库。
 * 提供传感器初始化、数据读取和控制功能，通过串口接口与传感器通信。
 *
 * 主要功能：
 * - 自动检测传感器波特率
 * - 读取加速度、陀螺仪、角度和磁力计数据
 * - 基于HAL库的串口通信
 *
 * 硬件要求：
 * - 支持串口外设的STM32微控制器
 * - WIT运动传感器(JY901S/JY901B或兼容型号)
 *
 * 引脚连接：
 * - UART_TX -> 传感器RX
 * - UART_RX -> 传感器TX
 * - VCC -> 3.3V或5V
 * - GND -> 地线
 *
 * 使用示例：
 * @code
 * // 初始化驱动
 * if(WIT_Driver_Init(115200) == WIT_HAL_OK) {
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

// 存储波特率对应的数值
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

// HAL库串口发送函数实现
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    HAL_UART_Transmit(&huart2, p_data, uiSize, 1000);
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
static uint32_t current_baud = 9600;

// 串口配置回调函数指针定义
typedef void (*UartConfigFunc)(uint32_t baudRate);

// 用于存储用户提供的串口配置回调函数
static UartConfigFunc uart_config_callback = NULL;

// 注册串口配置回调函数
void WIT_Driver_RegisterUartConfig(void (*configFunc)(uint32_t))
{
    uart_config_callback = configFunc;
}

// 内部函数声明
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static void AutoScanSensor(void);
static void ShowHelp(void);
static void CmdProcess(void);
static void CopeCmdData(unsigned char ucData);

// 驱动初始化函数
int32_t WIT_Driver_Init(uint32_t baudRate)
{
    current_baud = baudRate; // 保存当前波特率
    
    // 注意：这里假设主函数中已经进行了串口初始化
    // 如果需要自行初始化串口，可以在此处添加相应代码
    
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    // 注册HAL库串口发送函数
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);
    
    printf("\r\n********************** WIT-Motion UART Driver ************************\r\n");
    
    // 自动扫描设备（尝试不同波特率）
    AutoScanSensor();
    
    if(device_found)
    {
        printf("WIT sensor initialized successfully at baud rate: %lu\r\n", current_baud);
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
    
    // 发送查询命令获取传感器数据
    WitReadReg(AX, 12);
    CmdProcess(); // 处理命令
    
    // 检查是否有数据更新
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
        
        // 根据数据类型设置对应标志位
        wit_data.acc_updated = (s_cDataUpdate & ACC_UPDATE) ? 1 : 0;
        wit_data.gyro_updated = (s_cDataUpdate & GYRO_UPDATE) ? 1 : 0;
        wit_data.angle_updated = (s_cDataUpdate & ANGLE_UPDATE) ? 1 : 0;
        wit_data.mag_updated = (s_cDataUpdate & MAG_UPDATE) ? 1 : 0;
        
        // 清除更新标志，与Main.c不同，我们不在这里清除，以便用户可以查询哪些数据已更新
        // s_cDataUpdate = 0;
        
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
}

// 清除特定类型的数据更新标志
void WIT_Driver_ClearUpdateFlag(uint8_t flag_type)
{
    s_cDataUpdate &= ~flag_type;
    
    // 同时更新数据结构中的标志
    if(flag_type & ACC_UPDATE)
        wit_data.acc_updated = 0;
    if(flag_type & GYRO_UPDATE)
        wit_data.gyro_updated = 0;
    if(flag_type & ANGLE_UPDATE)
        wit_data.angle_updated = 0;
    if(flag_type & MAG_UPDATE)
        wit_data.mag_updated = 0;
}

// 处理外部命令
void WIT_Driver_ProcessCommand(char cmd)
{
    s_cCmd = cmd;
}

// 设置传感器带宽
int32_t WIT_Driver_SetBandwidth(uint8_t bandwidth)
{
    return WitSetBandwidth(bandwidth);
}

// 设置传感器波特率
int32_t WIT_Driver_SetBaudRate(uint8_t baudRate)
{
    int32_t ret = WitSetUartBaud(baudRate);
    if(ret == WIT_HAL_OK)
    {
        // 更新本地波特率记录
        current_baud = c_uiBaud[baudRate];
        
        // 如果有注册回调函数，则调用回调函数重新配置串口
        if(uart_config_callback != NULL)
        {
            uart_config_callback(current_baud);
        }
    }
    return ret;
}

// 设置传感器输出频率
int32_t WIT_Driver_SetOutputRate(uint8_t rate)
{
    return WitSetOutputRate(rate);
}

// 设置传感器输出内容
int32_t WIT_Driver_SetContent(uint8_t content)
{
    return WitSetContent(content);
}

// ===== 以下是从main.c移过来的内部函数 =====

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    for(int i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ: 
                s_cDataUpdate |= ACC_UPDATE;
                break;
            case GZ:
                s_cDataUpdate |= GYRO_UPDATE;
                break;
            case HZ:
                s_cDataUpdate |= MAG_UPDATE;
                break;
            case Yaw:
                s_cDataUpdate |= ANGLE_UPDATE;
                break;
            default:
                s_cDataUpdate |= READ_UPDATE;
                break;
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
    int i, iRetry;
    
    // 循环尝试不同的波特率
    for(i = 1; i < 10; i++) // 从1开始，跳过0索引
    {
        // 更新当前波特率
        current_baud = c_uiBaud[i];
        
        
        iRetry = 2;
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            HAL_Delay(100);
            if(s_cDataUpdate != 0)
            {
                printf("%lu baud find sensor\r\n\r\n", current_baud);
                device_found = 1;
                ShowHelp();
                return;
            }
            iRetry--;
        } while(iRetry);
    }
    
    printf("can not find sensor\r\n");
    printf("please check your connection\r\n");
    device_found = 0;
}

static void CopeCmdData(unsigned char ucData)
{
    static unsigned char s_ucData[50], s_ucRxCnt = 0;
    
    s_ucData[s_ucRxCnt++] = ucData;
    if(s_ucRxCnt < 3) return;                  // 少于三个数据返回
    if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
    if(s_ucRxCnt >= 3)
    {
        if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
        {
            s_cCmd = s_ucData[0];
            memset(s_ucData, 0, 50);
            s_ucRxCnt = 0;
        }
        else 
        {
            s_ucData[0] = s_ucData[1];
            s_ucData[1] = s_ucData[2];
            s_ucRxCnt = 2;
        }
    }
}

static void ShowHelp(void)
{
    printf("\r\n************************ WIT_DRIVER_HELP ************************\r\n");
    printf("Available commands:\r\n");
    printf("  'a' - Start acceleration calibration\r\n");
    printf("  'm' - Start magnetic calibration\r\n");
    printf("  'e' - End magnetic calibration\r\n");
    printf("  'U' - Bandwidth increase to 256Hz\r\n");
    printf("  'u' - Bandwidth reduction to 5Hz\r\n");
    printf("  'B' - Baud rate increased to 115200\r\n");
    printf("  'b' - Baud rate reduction to 9600\r\n");
    printf("  'R' - Return rate increases to 10Hz\r\n");
    printf("  'r' - Return rate reduction to 1Hz\r\n");
    printf("  'C' - Full content: acc, gyro, angle, mag\r\n");
    printf("  'c' - Basic content: acceleration only\r\n");
    printf("  'h' - Show this help\r\n");
    printf("******************************************************************\r\n");
}

static void CmdProcess(void)
{
    switch(s_cCmd)
    {
        case 'a':	
            if(WitStartAccCali() != WIT_HAL_OK) 
                printf("\r\nSet AccCali Error\r\n");
            break;
        case 'm':	
            if(WitStartMagCali() != WIT_HAL_OK) 
                printf("\r\nSet MagCali Error\r\n");
            break;
        case 'e':	
            if(WitStopMagCali() != WIT_HAL_OK)
                printf("\r\nSet MagCali Error\r\n");
            break;
        case 'u':	
            if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) 
                printf("\r\nSet Bandwidth Error\r\n");
            break;
        case 'U':	
            if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) 
                printf("\r\nSet Bandwidth Error\r\n");
            break;
        case 'B':	
            if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) 
                printf("\r\nSet Baud Error\r\n");
            else
                current_baud = c_uiBaud[WIT_BAUD_115200];
            break;
        case 'b':	
            if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
                printf("\r\nSet Baud Error\r\n");
            else
                current_baud = c_uiBaud[WIT_BAUD_9600];
            break;
        case 'R':	
            if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) 
                printf("\r\nSet Rate Error\r\n");
            break;
        case 'r':	
            if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) 
                printf("\r\nSet Rate Error\r\n");
            break;
        case 'C':	
            if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) 
                printf("\r\nSet RSW Error\r\n");
            break;
        case 'c':	
            if(WitSetContent(RSW_ACC) != WIT_HAL_OK) 
                printf("\r\nSet RSW Error\r\n");
            break;
        case 'h':	
            ShowHelp();
            break;
        default: 
            return;
    }
    s_cCmd = 0xff;
}