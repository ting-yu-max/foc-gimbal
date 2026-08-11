#ifndef AS5600_H
#define AS5600_H

#include "main.h"
#include "i2c.h"

#define AS5600_ADDR         (0x36 << 1)     /*7位地址0x36 ->HAL需8位，左移1位*/
#define AS5600_REG_RAW_H    0x0C            /*RAW_ANGLE 高字节寄存器地址*/
#define AS5600_RESOLUTION   4096.0f         /*12位分辨率*/
#define AS5600_2PI          6.28318530718f

uint16_t AS5600_GetRawAngle(void);          /*读取原始角度：0~4095*/
float   AS5600_GetAngleRad(void);

#endif
