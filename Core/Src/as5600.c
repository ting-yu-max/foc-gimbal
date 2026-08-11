#include "as5600.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdint.h>

/*读取AS5600原始角度数值（12位，0~4095）*/
uint16_t AS5600_GetRawAngle(void)
{
    uint8_t buf[2];
    HAL_StatusTypeDef ret;

    /*从寄存器0x0C起连续读取2字节，buf[0]高字节，buf[1]低字节*/
    ret = HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, AS5600_REG_RAW_H, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
    if (ret != HAL_OK)
    {
        return 0xFFFF;      /*I2C失败->返回无效值便于排查*/
    }

    /*高字节左移8位，拼上低字节，取低12位*/
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return raw & 0x0FFF;
}

/*转成机械角度（弧度：0~2pi)*/
float AS5600_GetAngleRad(void)
{
    return AS5600_GetRawAngle() *AS5600_2PI / AS5600_RESOLUTION;
}


