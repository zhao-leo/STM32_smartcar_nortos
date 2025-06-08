#include "icm42688.h"
#include "spi.h"
#include "MahonyAHRS.h"
#include <string.h>
#include <stdio.h>

#define correct_Time_define 1000    // 减少校准次数
#define SAMPLE_FREQ 1000.0f         // 采样频率1000Hz
#define TEMP_SENSITIVITY 132.48f    // 温度灵敏度

/*接口*/
/*用户CS接口*/
#define SPI_SCL3300_CS_LOW() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0)
#define	SPI_SCL3300_CS_HIGH() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1)

/*SPI底层函数接口*/
uint8_t hal_Spi2_ReadWriteByte(uint8_t txdata)
{
    uint8_t rxdata = 0;
    HAL_SPI_TransmitReceive(&hspi2, &txdata, &rxdata, 1, 5); /*halstm32的spi读写函数*/
    return rxdata;
}

/*变量*/
/*Mahony变量*/
uint8_t first_mahony=0; 
uint8_t device_id;
/*陀螺仪变量*/
float LSB_ACC_GYRO[2]={0};//陀螺仪数据转换变量
float GyroCal[3]={0};//陀螺仪校准值
float AccelCal[3]={0};//陀螺仪校准值
float GyroCorrected[3]={0};//已经校准完的值
float AccelCorrected[3]={0};//已经校准完的值
float icm42688_acc_x, icm42688_acc_y, icm42688_acc_z  ;// ICM42688加速度原始数据       
float icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z ; // ICM42688角速度原始速度数据
float icm42688_temp; // 温度数据
float gyro_correct[3]={0};
uint32_t correct_times=0;
uint8_t calibration_done = 0; // 校准完成标志
uint8_t system_initialized = 0; // 系统初始化标志

// 偏航角累加相关变量
float yaw_accumulated = 0.0f;
float last_yaw_rate = 0.0f;

/*面向ICM42688的spi读写函数封装*/
/*使用pBuffer进行数据交换*/
void Icm_Spi_ReadWriteNbytes(uint8_t* pBuffer, uint8_t len)
{
    uint8_t i = 0;
    for(i = 0; i < len; i ++)
    {
		*pBuffer = hal_Spi2_ReadWriteByte(*pBuffer);
        pBuffer++;
    }

}
 
/*******************************************************************************
* 名    称： icm42688_read_reg
* 功    能： 读取单个寄存器的值
* 入口参数： reg: 寄存器地址
* 出口参数： 当前寄存器地址的值
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page51.
*******************************************************************************/
uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t regval = 0;
    SPI_SCL3300_CS_LOW();
    reg |= 0x80;
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg, 1);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(&regval, 1);
    SPI_SCL3300_CS_HIGH();
    return regval;
}
 
/*******************************************************************************
* 名    称： icm42688_read_regs
* 功    能： 连续读取多个寄存器的值
* 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{

    reg |= 0x80;
    SPI_SCL3300_CS_LOW();
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg, 1);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(buf, len);
    SPI_SCL3300_CS_HIGH();
	HAL_Delay(1);

}
 
 
/*******************************************************************************
* 名    称： icm42688_write_reg
* 功    能： 向单个寄存器写数据
* 入口参数： reg: 寄存器地址 value:数据
* 出口参数： 0
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
{

    SPI_SCL3300_CS_LOW();
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg, 1);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(&value, 1);
    SPI_SCL3300_CS_HIGH();
	HAL_Delay(1); // 确保CS高电平稳定
    return 0;
}


/*另一个spi读取函数*/
/*addr为ICM42688寄存器地址,dat为接收的数据*/
void icm42688_readReg(uint8_t addr, uint8_t *dat)
{
	*dat=icm42688_read_reg(addr);
}
/*另一个spi写函数*/
/*addr为ICM42688寄存器地址,dat为要写入的数据*/
void icm42688_writeReg(uint8_t addr, uint8_t dat)
{
	icm42688_write_reg(addr,dat);
}


/*ICM42688的初始化*/
int8_t Icm42688_Init(void)
{

	/*指定Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*软重启*/
	icm42688_writeReg(0x11,0x01);
	HAL_Delay(10); // 等待重启完成

    // 检查设备ID
    device_id = icm42688_read_reg(ICM42688_WHO_AM_I);
    if(device_id != ICM42688_ID) {
        printf("ICM42688 not found! ID: 0x%02X\r\n", device_id);
    }

	/*读取中断位 切换SPI*/
//	buf = IMU->ReadReg(0x2D);
	/*指定Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*Gyro设置*/
	icm42688_writeReg(0x4F,0x06);//2000dps 1KHz
	/*Accel设置*/
	icm42688_writeReg(0x50,0x06);//16G 1KHz
	/*电源管理*/
	icm42688_writeReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
	
	HAL_Delay(10);
	
	/*指定Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*中断输出设置*/
//	icm42688_writeReg(0x14,0x12);//INT1 INT2 脉冲模式，低有效
	/*Gyro设置*/
	icm42688_writeReg(0x4F,0x06);//2000dps 1KHz
	/*Accel设置*/
	icm42688_writeReg(0x50,0x06);//16G 1KHz
	/*LSB设置*/
	LSB_ACC_GYRO[0] = LSB_ACC_16G;
	LSB_ACC_GYRO[1] = LSB_GYRO_2000_R;
	/*Tem设置&Gyro_Config1*/
	icm42688_writeReg(0x51,0x56);//BW 82Hz Latency = 2ms
	/*GYRO_ACCEL_CONFIG0*/
	icm42688_writeReg(0x52,0x11);//1BW
	/*ACCEL_CONFIG1*/
	icm42688_writeReg(0x53,0x0D);//Null
	/*INT_CONFIG0*/
	icm42688_writeReg(0x63,0x00);//Null
	/*INT_CONFIG1*/
//	icm42688_writeReg(0x64,0x00);//中断引脚正常启用
	/*INT_SOURCE0*/
	icm42688_writeReg(0x65,0x08);//DRDY INT1
	/*INT_SOURCE1*/
	icm42688_writeReg(0x66,0x00);//Null
	/*INT_SOURCE3*/
	icm42688_writeReg(0x68,0x00);//Null
	/*INT_SOURCE3*/
	icm42688_writeReg(0x69,0x00);//Null
	
/*****抗混叠滤波器@536Hz*****/
	
	/*GYRO抗混叠滤波器配置*/
	/*指定Bank1*/
	icm42688_writeReg(0x76,0x01);
	/*GYRO抗混叠滤波器配置*/
	icm42688_writeReg(0x0B,0xA0);//开启抗混叠和陷波滤波器
	icm42688_writeReg(0x0C,0x0C);//GYRO_AAF_DELT 12 (default 13)
	icm42688_writeReg(0x0D,0x90);//GYRO_AAF_DELTSQR 144 (default 170)
	icm42688_writeReg(0x0E,0x80);//GYRO_AAF_BITSHIFT 8 (default 8)
	
	/*ACCEL抗混叠滤波器配置*/
	/*指定Bank2*/
	icm42688_writeReg(0x76,0x02);
	/*ACCEL抗混叠滤波器配置*/
	icm42688_writeReg(0x03,0x18);//开启滤波器 ACCEL_AFF_DELT 12 (default 24)
	icm42688_writeReg(0x04,0x90);//ACCEL_AFF_DELTSQR 144 (default 64)
	icm42688_writeReg(0x05,0x80);//ACCEL_AAF_BITSHIFT 8 (default 6)

/*****自定义滤波器1号@111Hz*****/

	/*指定Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*滤波器顺序*/
	icm42688_writeReg(0x51,0x12);//GYRO滤波器1st
	icm42688_writeReg(0x53,0x05);//ACCEL滤波器1st
	/*滤波器设置*/
	icm42688_writeReg(0x52,0x33);//111Hz 03
	/*指定Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*电源管理*/
	icm42688_writeReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
	HAL_Delay(10);
	return 0;
}


/*获取ICM42688的加速度原始值*/
void Get_Acc_ICM42688(void)
{
    unsigned char dat[6];
	icm42688_read_regs(ICM42688_ACCEL_DATA_X1,dat,6);
    icm42688_acc_x = (short int)(((short int)dat[0] << 8) | dat[1]);
    icm42688_acc_y = (short int)(((short int)dat[2] << 8) | dat[3]);
    icm42688_acc_z = (short int)(((short int)dat[4] << 8) | dat[5]);

}
/*获取ICM42688的角速度原始值*/
void Get_Gyro_ICM42688(void)
{
    unsigned char dat[6];
	icm42688_read_regs(ICM42688_GYRO_DATA_X1,dat,6);
    icm42688_gyro_x = (short int)(((short int)dat[0] << 8) | dat[1]);
    icm42688_gyro_y = (short int)(((short int)dat[2] << 8) | dat[3]);
    icm42688_gyro_z = (short int)(((short int)dat[4] << 8) | dat[5]);
}

/*获取ICM42688的温度值*/
void Get_Temp_ICM42688(void)
{
    unsigned char dat[2];
    icm42688_read_regs(ICM42688_TEMP_DATA1, dat, 2);
    int16_t temp_raw = (int16_t)(((int16_t)dat[0] << 8) | dat[1]);
    icm42688_temp = (float)temp_raw / TEMP_SENSITIVITY + 25.0f; // 转换为摄氏度
}

/*陀螺仪校准函数 - 简化版本*/
void Icm42688_Calibrate(void)
{
    printf("开始陀螺仪校准，请保持设备静止...\r\n");
    
    // 重置校准值
    gyro_correct[0] = gyro_correct[1] = gyro_correct[2] = 0;
    correct_times = 0;
    
    // 收集校准数据
    for(int i = 0; i < correct_Time_define; i++)
    {
        Get_Gyro_ICM42688();
        gyro_correct[0] += ((float)icm42688_gyro_x * LSB_ACC_GYRO[1]);
        gyro_correct[1] += ((float)icm42688_gyro_y * LSB_ACC_GYRO[1]);
        gyro_correct[2] += ((float)icm42688_gyro_z * LSB_ACC_GYRO[1]);
        HAL_Delay(1); // 1ms延时
    }
    
    // 计算平均值作为零偏
    gyro_correct[0] /= correct_Time_define;
    gyro_correct[1] /= correct_Time_define;
    gyro_correct[2] /= correct_Time_define;
    
    calibration_done = 1;
    printf("陀螺仪校准完成! 零偏: X=%.4f, Y=%.4f, Z=%.4f\r\n", 
           gyro_correct[0], gyro_correct[1], gyro_correct[2]);
}

/*系统重置函数*/
void Icm42688_Reset(void)
{
    printf("重置ICM42688姿态解算系统...\r\n");
    
    // 重置所有状态变量
    calibration_done = 0;
    system_initialized = 0;
    correct_times = 0;
    gyro_correct[0] = gyro_correct[1] = gyro_correct[2] = 0;
    yaw_accumulated = 0.0f;
    last_yaw_rate = 0.0f;
    
    // 重置Mahony算法
    Mahony_reset();
    
    printf("系统重置完成\r\n");
}

/*系统初始化函数*/
void Icm42688_SystemInit(void)
{
    if(!system_initialized)
    {
        printf("初始化ICM42688姿态解算系统...\r\n");
        
        // 首先重置Mahony滤波器
        Mahony_reset();
        
        // 初始化Mahony算法
        Mahony_Init(SAMPLE_FREQ);
        
        // 执行陀螺仪校准
        Icm42688_Calibrate();
        
        // 获取初始加速度计数据用于姿态初始化
        Get_Acc_ICM42688();
        AccelCorrected[0] = ((float)icm42688_acc_x * LSB_ACC_GYRO[0]);
        AccelCorrected[1] = ((float)icm42688_acc_y * LSB_ACC_GYRO[0]);
        AccelCorrected[2] = ((float)icm42688_acc_z * LSB_ACC_GYRO[0]);
        
        // 初始化Mahony算法的四元数
        MahonyAHRSinit(AccelCorrected[0], AccelCorrected[1], AccelCorrected[2], 0, 0, 0);
        
        system_initialized = 1;
        printf("ICM42688系统初始化完成\r\n");
    }
}

/*对ICM42688进行数据处理解算出角度 - 优化版本*/
void Get_MahonyAngle(float* Roll, float* Pitch, float * Yaw)
{
    // 如果系统未初始化，先初始化
    if(!system_initialized)
    {
        Icm42688_SystemInit();
        *Roll = 0.0f;
        *Pitch = 0.0f;
        *Yaw = 0.0f;
        return;
    }
    
    // 获取传感器数据
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();
    Get_Temp_ICM42688();
    
    // 转换为标准单位
    GyroCorrected[0] = ((float)icm42688_gyro_x * LSB_ACC_GYRO[1]);
    GyroCorrected[1] = ((float)icm42688_gyro_y * LSB_ACC_GYRO[1]);
    GyroCorrected[2] = ((float)icm42688_gyro_z * LSB_ACC_GYRO[1]);
    
    AccelCorrected[0] = ((float)icm42688_acc_x * LSB_ACC_GYRO[0]);
    AccelCorrected[1] = ((float)icm42688_acc_y * LSB_ACC_GYRO[0]);
    AccelCorrected[2] = ((float)icm42688_acc_z * LSB_ACC_GYRO[0]);
    
    // 减去陀螺仪零偏
    GyroCorrected[0] -= gyro_correct[0];
    GyroCorrected[1] -= gyro_correct[1];
    GyroCorrected[2] -= gyro_correct[2];
    
    // 使用Mahony算法更新俯仰角和翻滚角（不使用磁力计）
    MahonyAHRSupdateIMU(GyroCorrected[0], GyroCorrected[1], GyroCorrected[2],
                        AccelCorrected[0], AccelCorrected[1], AccelCorrected[2]);
    
    // 计算俯仰角和翻滚角（使用滤波）
    Mahony_computeAngles();
    
    // 偏航角使用积分累加方式
    float current_yaw_rate = GyroCorrected[2]; // Z轴角速度
    
    // 简单的低通滤波去除高频噪声
    float alpha = 0.9f; // 滤波系数
    current_yaw_rate = alpha * last_yaw_rate + (1.0f - alpha) * current_yaw_rate;
    last_yaw_rate = current_yaw_rate;
    
    // 积分计算偏航角（弧度转角度）
    yaw_accumulated += current_yaw_rate * (1.0f / SAMPLE_FREQ) * 57.29578f;
    
    // 限制偏航角范围 -180 到 180 度
    while(yaw_accumulated > 180.0f) yaw_accumulated -= 360.0f;
    while(yaw_accumulated < -180.0f) yaw_accumulated += 360.0f;
    
    // 输出结果
    *Roll = roll_mahony;
    *Pitch = pitch_mahony;
    *Yaw = yaw_accumulated;
}

