#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <stdint.h>
#include "driver/i2c.h"

// I2C Manager Initialization
esp_err_t i2c_manager_init(void);

// I2C Read/Write Funcitons
esp_err_t i2c_manager_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_manager_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_manager_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
esp_err_t i2c_manager_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);


#endif   // I2C_MANAGER_H