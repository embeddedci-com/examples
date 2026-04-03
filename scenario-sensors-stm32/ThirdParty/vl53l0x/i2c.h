#ifndef VL53L0X_PORT_I2C_H_
#define VL53L0X_PORT_I2C_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

void vl53l0x_port_set_i2c(I2C_HandleTypeDef *hi2c);
uint8_t i2c_write(uint8_t addr_7b, uint8_t *buf, uint8_t len);
uint8_t i2c_read(uint8_t addr_7b, uint8_t *buf, uint8_t len);

#endif
