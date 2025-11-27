#include "bsp_imu.h"
#include "i2c.h"

// Private variables
static MPU6050_Data_t imu_data;
static volatile IMU_State_t imu_state = IMU_IDLE;
static uint8_t dma_rx_buffer[14]; // Raw buffer for DMA to fill

// Helper for blocking writes (only used during Init)
static void MPU_Write(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, 1, &value, 1, 100);
}

/**
 * @brief  Wake up MPU6050 and configure ranges.
 * @return 1 if ID match, 0 if error
 */
uint8_t BSP_IMU_Init(void) {
    uint8_t check = 0;

    // 1. Check WHO_AM_I (Blocking Read)
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_WHO_AM_I, 1, &check, 1, 100);

    if (check == 0x68) { // 0x68 is the default ID
        // 2. Wake up sensor (Clear Sleep bit)
        MPU_Write(REG_PWR_MGMT_1, 0x00);

        // 3. Set Sample Rate Divider (1kHz output)
        MPU_Write(REG_SMPLRT_DIV, 0x07);

        // 4. Configure Accelerometer (+/- 2g)
        MPU_Write(REG_ACCEL_CONFIG, 0x00);

        // 5. Configure Gyro (+/- 250 dps)
        MPU_Write(REG_GYRO_CONFIG, 0x00);

        return 1; // OK
    }
    return 0; // Device not found
}

/**
 * @brief  Triggers a non-blocking DMA read of 14 bytes.
 */
void BSP_IMU_Start_Read_DMA(void) {
    if(imu_state == IMU_READING) return; // Busy protection

    imu_state = IMU_READING;

    // Request 14 bytes starting from ACCEL_XOUT_H
    HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, REG_ACCEL_XOUT_H, 1, dma_rx_buffer, 14);
}

/**
 * @brief  Called automatically by HAL when DMA finishes transfer.
 * We use this to parse the raw bytes into integers.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {

        imu_data.Accel_X_RAW = (int16_t)(dma_rx_buffer[0] << 8 | dma_rx_buffer[1]);
        imu_data.Accel_Y_RAW = (int16_t)(dma_rx_buffer[2] << 8 | dma_rx_buffer[3]);
        imu_data.Accel_Z_RAW = (int16_t)(dma_rx_buffer[4] << 8 | dma_rx_buffer[5]);
        imu_data.Temp_RAW    = (int16_t)(dma_rx_buffer[6] << 8 | dma_rx_buffer[7]);
        imu_data.Gyro_X_RAW  = (int16_t)(dma_rx_buffer[8] << 8 | dma_rx_buffer[9]);
        imu_data.Gyro_Y_RAW  = (int16_t)(dma_rx_buffer[10] << 8 | dma_rx_buffer[11]);
        imu_data.Gyro_Z_RAW  = (int16_t)(dma_rx_buffer[12] << 8 | dma_rx_buffer[13]);

        imu_state = IMU_IDLE; // Ready for next read
    }
}

MPU6050_Data_t BSP_IMU_Get_Data(void) {
    return imu_data;
}

IMU_State_t BSP_IMU_Get_State(void) {
    return imu_state;
}
