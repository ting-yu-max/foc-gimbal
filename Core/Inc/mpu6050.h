#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include "i2c.h"

#define MPU6050_ADDR        0xD0
#define MPU6050_PWR         0x6B
#define MPU6050_ACCEL           0x3B
#define MPU6050_GYRO           0x43

void MPU6050_Init(void);
void MPU6050_Read(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

#endif
