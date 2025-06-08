/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "i2c.h"

// MPU6050 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;

    float YawAngle;      // Z轴累计角度(偏航角)
    float YawDiff;       // 偏航角差值
    float PrevYawAngle;  // 上一次的偏航角
} MPU6050_t;


// 陀螺仪角度累计计算结构体
typedef struct {
    float currentAngle;        // 当前累积角度
    float previousAngle;       // 上一次的累积角度
    float angleDiff;           // 两次角度之间的差值
    float gyroZFiltered;       // 滤波后的z轴角速度
    float noiseThreshold;      // 噪声阈值
    uint32_t lastTime;         // 上次更新时间
    uint8_t isInitialized;     // 是否初始化
} GyroIntegrator_t;


// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

void GyroIntegrator_Init(GyroIntegrator_t *integrator, float noiseThreshold);
void GyroIntegrator_Update(GyroIntegrator_t *integrator, float gyroZ, uint8_t resetFlag);
void MPU6050_Reset_YawAngle(MPU6050_t *DataStruct);
