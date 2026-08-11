#include "mpu6050.h"
#include "i2c.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdint.h>

void MPU6050_Init()
{
    uint8_t data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1,  MPU6050_ADDR,  MPU6050_PWR,  1,  &data,  1, 100);
}

void MPU6050_Read(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) 
{
    uint8_t buf[6];

    /*加速度*/
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL, 1, buf, 6, 100);
    *ax = (int16_t) ((buf[0] << 8) | buf[1]);
    *ay = (int16_t) ((buf[2] << 8) | buf[3]);
    *az = (int16_t) ((buf[4] << 8) | buf[5]);

    /*陀螺仪*/
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO, 1, buf, 6, 100);
    *gx = (int16_t) ((buf[0] << 8) | buf[1]);
    *gy = (int16_t) ((buf[2] << 8) | buf[3]);
    *gz = (int16_t) ((buf[4] << 8) | buf[5]);
}
