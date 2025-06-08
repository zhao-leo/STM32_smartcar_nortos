#include "icm42688.h"
#include "spi.h"
#include "MahonyAHRS.h"
#include <string.h>
#include <stdio.h>

#define correct_Time_define 1000    // ����У׼����
#define SAMPLE_FREQ 1000.0f         // ����Ƶ��1000Hz
#define TEMP_SENSITIVITY 132.48f    // �¶�������

/*�ӿ�*/
/*�û�CS�ӿ�*/
#define SPI_SCL3300_CS_LOW() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0)
#define	SPI_SCL3300_CS_HIGH() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1)

/*SPI�ײ㺯���ӿ�*/
uint8_t hal_Spi2_ReadWriteByte(uint8_t txdata)
{
    uint8_t rxdata = 0;
    HAL_SPI_TransmitReceive(&hspi2, &txdata, &rxdata, 1, 5); /*halstm32��spi��д����*/
    return rxdata;
}

/*����*/
/*Mahony����*/
uint8_t first_mahony=0; 
uint8_t device_id;
/*�����Ǳ���*/
float LSB_ACC_GYRO[2]={0};//����������ת������
float GyroCal[3]={0};//������У׼ֵ
float AccelCal[3]={0};//������У׼ֵ
float GyroCorrected[3]={0};//�Ѿ�У׼���ֵ
float AccelCorrected[3]={0};//�Ѿ�У׼���ֵ
float icm42688_acc_x, icm42688_acc_y, icm42688_acc_z  ;// ICM42688���ٶ�ԭʼ����       
float icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z ; // ICM42688���ٶ�ԭʼ�ٶ�����
float icm42688_temp; // �¶�����
float gyro_correct[3]={0};
uint32_t correct_times=0;
uint8_t calibration_done = 0; // У׼��ɱ�־
uint8_t system_initialized = 0; // ϵͳ��ʼ����־

// ƫ�����ۼ���ر���
float yaw_accumulated = 0.0f;
float last_yaw_rate = 0.0f;

/*����ICM42688��spi��д������װ*/
/*ʹ��pBuffer�������ݽ���*/
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
* ��    �ƣ� icm42688_read_reg
* ��    �ܣ� ��ȡ�����Ĵ�����ֵ
* ��ڲ����� reg: �Ĵ�����ַ
* ���ڲ����� ��ǰ�Ĵ�����ַ��ֵ
* �������ߣ� Baxiange
* �������ڣ� 2022-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע�� ʹ��SPI��ȡ�Ĵ���ʱҪע��:���λΪ��дλ�����datasheet page51.
*******************************************************************************/
uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t regval = 0;
    SPI_SCL3300_CS_LOW();
    reg |= 0x80;
    /* д��Ҫ���ļĴ�����ַ */
    Icm_Spi_ReadWriteNbytes(&reg, 1);
    /* ��ȡ�Ĵ������� */
    Icm_Spi_ReadWriteNbytes(&regval, 1);
    SPI_SCL3300_CS_HIGH();
    return regval;
}
 
/*******************************************************************************
* ��    �ƣ� icm42688_read_regs
* ��    �ܣ� ������ȡ����Ĵ�����ֵ
* ��ڲ����� reg: ��ʼ�Ĵ�����ַ *buf����ָ��,uint16_t len����
* ���ڲ����� ��
* �������ߣ� Baxiange
* �������ڣ� 2022-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע�� ʹ��SPI��ȡ�Ĵ���ʱҪע��:���λΪ��дλ�����datasheet page50.
*******************************************************************************/
void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{

    reg |= 0x80;
    SPI_SCL3300_CS_LOW();
    /* д��Ҫ���ļĴ�����ַ */
    Icm_Spi_ReadWriteNbytes(&reg, 1);
    /* ��ȡ�Ĵ������� */
    Icm_Spi_ReadWriteNbytes(buf, len);
    SPI_SCL3300_CS_HIGH();
	HAL_Delay(1);

}
 
 
/*******************************************************************************
* ��    �ƣ� icm42688_write_reg
* ��    �ܣ� �򵥸��Ĵ���д����
* ��ڲ����� reg: �Ĵ�����ַ value:����
* ���ڲ����� 0
* �������ߣ� Baxiange
* �������ڣ� 2022-07-25
* ��    �ģ�
* �޸����ڣ�
* ��    ע�� ʹ��SPI��ȡ�Ĵ���ʱҪע��:���λΪ��дλ�����datasheet page50.
*******************************************************************************/
uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
{

    SPI_SCL3300_CS_LOW();
    /* д��Ҫ���ļĴ�����ַ */
    Icm_Spi_ReadWriteNbytes(&reg, 1);
    /* ��ȡ�Ĵ������� */
    Icm_Spi_ReadWriteNbytes(&value, 1);
    SPI_SCL3300_CS_HIGH();
	HAL_Delay(1); // ȷ��CS�ߵ�ƽ�ȶ�
    return 0;
}


/*��һ��spi��ȡ����*/
/*addrΪICM42688�Ĵ�����ַ,datΪ���յ�����*/
void icm42688_readReg(uint8_t addr, uint8_t *dat)
{
	*dat=icm42688_read_reg(addr);
}
/*��һ��spiд����*/
/*addrΪICM42688�Ĵ�����ַ,datΪҪд�������*/
void icm42688_writeReg(uint8_t addr, uint8_t dat)
{
	icm42688_write_reg(addr,dat);
}


/*ICM42688�ĳ�ʼ��*/
int8_t Icm42688_Init(void)
{

	/*ָ��Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*������*/
	icm42688_writeReg(0x11,0x01);
	HAL_Delay(10); // �ȴ��������

    // ����豸ID
    device_id = icm42688_read_reg(ICM42688_WHO_AM_I);
    if(device_id != ICM42688_ID) {
        printf("ICM42688 not found! ID: 0x%02X\r\n", device_id);
    }

	/*��ȡ�ж�λ �л�SPI*/
//	buf = IMU->ReadReg(0x2D);
	/*ָ��Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*Gyro����*/
	icm42688_writeReg(0x4F,0x06);//2000dps 1KHz
	/*Accel����*/
	icm42688_writeReg(0x50,0x06);//16G 1KHz
	/*��Դ����*/
	icm42688_writeReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
	
	HAL_Delay(10);
	
	/*ָ��Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*�ж��������*/
//	icm42688_writeReg(0x14,0x12);//INT1 INT2 ����ģʽ������Ч
	/*Gyro����*/
	icm42688_writeReg(0x4F,0x06);//2000dps 1KHz
	/*Accel����*/
	icm42688_writeReg(0x50,0x06);//16G 1KHz
	/*LSB����*/
	LSB_ACC_GYRO[0] = LSB_ACC_16G;
	LSB_ACC_GYRO[1] = LSB_GYRO_2000_R;
	/*Tem����&Gyro_Config1*/
	icm42688_writeReg(0x51,0x56);//BW 82Hz Latency = 2ms
	/*GYRO_ACCEL_CONFIG0*/
	icm42688_writeReg(0x52,0x11);//1BW
	/*ACCEL_CONFIG1*/
	icm42688_writeReg(0x53,0x0D);//Null
	/*INT_CONFIG0*/
	icm42688_writeReg(0x63,0x00);//Null
	/*INT_CONFIG1*/
//	icm42688_writeReg(0x64,0x00);//�ж�������������
	/*INT_SOURCE0*/
	icm42688_writeReg(0x65,0x08);//DRDY INT1
	/*INT_SOURCE1*/
	icm42688_writeReg(0x66,0x00);//Null
	/*INT_SOURCE3*/
	icm42688_writeReg(0x68,0x00);//Null
	/*INT_SOURCE3*/
	icm42688_writeReg(0x69,0x00);//Null
	
/*****������˲���@536Hz*****/
	
	/*GYRO������˲�������*/
	/*ָ��Bank1*/
	icm42688_writeReg(0x76,0x01);
	/*GYRO������˲�������*/
	icm42688_writeReg(0x0B,0xA0);//������������ݲ��˲���
	icm42688_writeReg(0x0C,0x0C);//GYRO_AAF_DELT 12 (default 13)
	icm42688_writeReg(0x0D,0x90);//GYRO_AAF_DELTSQR 144 (default 170)
	icm42688_writeReg(0x0E,0x80);//GYRO_AAF_BITSHIFT 8 (default 8)
	
	/*ACCEL������˲�������*/
	/*ָ��Bank2*/
	icm42688_writeReg(0x76,0x02);
	/*ACCEL������˲�������*/
	icm42688_writeReg(0x03,0x18);//�����˲��� ACCEL_AFF_DELT 12 (default 24)
	icm42688_writeReg(0x04,0x90);//ACCEL_AFF_DELTSQR 144 (default 64)
	icm42688_writeReg(0x05,0x80);//ACCEL_AAF_BITSHIFT 8 (default 6)

/*****�Զ����˲���1��@111Hz*****/

	/*ָ��Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*�˲���˳��*/
	icm42688_writeReg(0x51,0x12);//GYRO�˲���1st
	icm42688_writeReg(0x53,0x05);//ACCEL�˲���1st
	/*�˲�������*/
	icm42688_writeReg(0x52,0x33);//111Hz 03
	/*ָ��Bank0*/
	icm42688_writeReg(0x76,0x00);
	/*��Դ����*/
	icm42688_writeReg(0x4E,0x0F);//ACC GYRO LowNoise Mode
	HAL_Delay(10);
	return 0;
}


/*��ȡICM42688�ļ��ٶ�ԭʼֵ*/
void Get_Acc_ICM42688(void)
{
    unsigned char dat[6];
	icm42688_read_regs(ICM42688_ACCEL_DATA_X1,dat,6);
    icm42688_acc_x = (short int)(((short int)dat[0] << 8) | dat[1]);
    icm42688_acc_y = (short int)(((short int)dat[2] << 8) | dat[3]);
    icm42688_acc_z = (short int)(((short int)dat[4] << 8) | dat[5]);

}
/*��ȡICM42688�Ľ��ٶ�ԭʼֵ*/
void Get_Gyro_ICM42688(void)
{
    unsigned char dat[6];
	icm42688_read_regs(ICM42688_GYRO_DATA_X1,dat,6);
    icm42688_gyro_x = (short int)(((short int)dat[0] << 8) | dat[1]);
    icm42688_gyro_y = (short int)(((short int)dat[2] << 8) | dat[3]);
    icm42688_gyro_z = (short int)(((short int)dat[4] << 8) | dat[5]);
}

/*��ȡICM42688���¶�ֵ*/
void Get_Temp_ICM42688(void)
{
    unsigned char dat[2];
    icm42688_read_regs(ICM42688_TEMP_DATA1, dat, 2);
    int16_t temp_raw = (int16_t)(((int16_t)dat[0] << 8) | dat[1]);
    icm42688_temp = (float)temp_raw / TEMP_SENSITIVITY + 25.0f; // ת��Ϊ���϶�
}

/*������У׼���� - �򻯰汾*/
void Icm42688_Calibrate(void)
{
    printf("��ʼ������У׼���뱣���豸��ֹ...\r\n");
    
    // ����У׼ֵ
    gyro_correct[0] = gyro_correct[1] = gyro_correct[2] = 0;
    correct_times = 0;
    
    // �ռ�У׼����
    for(int i = 0; i < correct_Time_define; i++)
    {
        Get_Gyro_ICM42688();
        gyro_correct[0] += ((float)icm42688_gyro_x * LSB_ACC_GYRO[1]);
        gyro_correct[1] += ((float)icm42688_gyro_y * LSB_ACC_GYRO[1]);
        gyro_correct[2] += ((float)icm42688_gyro_z * LSB_ACC_GYRO[1]);
        HAL_Delay(1); // 1ms��ʱ
    }
    
    // ����ƽ��ֵ��Ϊ��ƫ
    gyro_correct[0] /= correct_Time_define;
    gyro_correct[1] /= correct_Time_define;
    gyro_correct[2] /= correct_Time_define;
    
    calibration_done = 1;
    printf("������У׼���! ��ƫ: X=%.4f, Y=%.4f, Z=%.4f\r\n", 
           gyro_correct[0], gyro_correct[1], gyro_correct[2]);
}

/*ϵͳ���ú���*/
void Icm42688_Reset(void)
{
    printf("����ICM42688��̬����ϵͳ...\r\n");
    
    // ��������״̬����
    calibration_done = 0;
    system_initialized = 0;
    correct_times = 0;
    gyro_correct[0] = gyro_correct[1] = gyro_correct[2] = 0;
    yaw_accumulated = 0.0f;
    last_yaw_rate = 0.0f;
    
    // ����Mahony�㷨
    Mahony_reset();
    
    printf("ϵͳ�������\r\n");
}

/*ϵͳ��ʼ������*/
void Icm42688_SystemInit(void)
{
    if(!system_initialized)
    {
        printf("��ʼ��ICM42688��̬����ϵͳ...\r\n");
        
        // ��������Mahony�˲���
        Mahony_reset();
        
        // ��ʼ��Mahony�㷨
        Mahony_Init(SAMPLE_FREQ);
        
        // ִ��������У׼
        Icm42688_Calibrate();
        
        // ��ȡ��ʼ���ٶȼ�����������̬��ʼ��
        Get_Acc_ICM42688();
        AccelCorrected[0] = ((float)icm42688_acc_x * LSB_ACC_GYRO[0]);
        AccelCorrected[1] = ((float)icm42688_acc_y * LSB_ACC_GYRO[0]);
        AccelCorrected[2] = ((float)icm42688_acc_z * LSB_ACC_GYRO[0]);
        
        // ��ʼ��Mahony�㷨����Ԫ��
        MahonyAHRSinit(AccelCorrected[0], AccelCorrected[1], AccelCorrected[2], 0, 0, 0);
        
        system_initialized = 1;
        printf("ICM42688ϵͳ��ʼ�����\r\n");
    }
}

/*��ICM42688�������ݴ��������Ƕ� - �Ż��汾*/
void Get_MahonyAngle(float* Roll, float* Pitch, float * Yaw)
{
    // ���ϵͳδ��ʼ�����ȳ�ʼ��
    if(!system_initialized)
    {
        Icm42688_SystemInit();
        *Roll = 0.0f;
        *Pitch = 0.0f;
        *Yaw = 0.0f;
        return;
    }
    
    // ��ȡ����������
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();
    Get_Temp_ICM42688();
    
    // ת��Ϊ��׼��λ
    GyroCorrected[0] = ((float)icm42688_gyro_x * LSB_ACC_GYRO[1]);
    GyroCorrected[1] = ((float)icm42688_gyro_y * LSB_ACC_GYRO[1]);
    GyroCorrected[2] = ((float)icm42688_gyro_z * LSB_ACC_GYRO[1]);
    
    AccelCorrected[0] = ((float)icm42688_acc_x * LSB_ACC_GYRO[0]);
    AccelCorrected[1] = ((float)icm42688_acc_y * LSB_ACC_GYRO[0]);
    AccelCorrected[2] = ((float)icm42688_acc_z * LSB_ACC_GYRO[0]);
    
    // ��ȥ��������ƫ
    GyroCorrected[0] -= gyro_correct[0];
    GyroCorrected[1] -= gyro_correct[1];
    GyroCorrected[2] -= gyro_correct[2];
    
    // ʹ��Mahony�㷨���¸����Ǻͷ����ǣ���ʹ�ô����ƣ�
    MahonyAHRSupdateIMU(GyroCorrected[0], GyroCorrected[1], GyroCorrected[2],
                        AccelCorrected[0], AccelCorrected[1], AccelCorrected[2]);
    
    // ���㸩���Ǻͷ����ǣ�ʹ���˲���
    Mahony_computeAngles();
    
    // ƫ����ʹ�û����ۼӷ�ʽ
    float current_yaw_rate = GyroCorrected[2]; // Z����ٶ�
    
    // �򵥵ĵ�ͨ�˲�ȥ����Ƶ����
    float alpha = 0.9f; // �˲�ϵ��
    current_yaw_rate = alpha * last_yaw_rate + (1.0f - alpha) * current_yaw_rate;
    last_yaw_rate = current_yaw_rate;
    
    // ���ּ���ƫ���ǣ�����ת�Ƕȣ�
    yaw_accumulated += current_yaw_rate * (1.0f / SAMPLE_FREQ) * 57.29578f;
    
    // ����ƫ���Ƿ�Χ -180 �� 180 ��
    while(yaw_accumulated > 180.0f) yaw_accumulated -= 360.0f;
    while(yaw_accumulated < -180.0f) yaw_accumulated += 360.0f;
    
    // ������
    *Roll = roll_mahony;
    *Pitch = pitch_mahony;
    *Yaw = yaw_accumulated;
}

