/*
 * bsp_imu.h
 * MPU6050 Driver using I2C DMA for STM32F446RE
 */

#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "stm32f4xx_hal.h"

#define MPU6050_ADDR         (0x68 << 1)

// Register Map
#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_PWR_MGMT_1       0x6B
#define REG_WHO_AM_I         0x75
#define REG_ACCEL_XOUT_H     0x3B

// Raw Data Structure
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Temp_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
} MPU6050_Data_t;

// Driver State
typedef enum {
    IMU_IDLE = 0,
    IMU_READING,
    IMU_ERROR
} IMU_State_t;

// Prototypes
uint8_t BSP_IMU_Init(void);
void BSP_IMU_Start_Read_DMA(void);
MPU6050_Data_t BSP_IMU_Get_Data(void);
IMU_State_t BSP_IMU_Get_State(void);
void I2C1_BusRecovery(void);

#endif /* BSP_IMU_H */
