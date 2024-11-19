#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <cstddef>
#include <cstdint>
#include <stdint.h>
#include <stdbool.h>
#include "../config.h"
#include <driver/i2c.h>

// I2C Configuration 
#define I2C_PORT           I2C_PORT_0
#define I2C_TIMEOUT        1000
#define I2C_FREQ_HZ        400000     // 400 KHz

//Error codes
typedef enum {
    I2C_MANAGER_OK = 0,
    I2C_MANAGER_INIT_ERROR,
    I2C_MANAGER_WRITE_ERROR,
    I2C_MANAGER_READ_ERROR,
    I2C_MANAGER_ACK_ERROR,
    I2C_MANAGER_TIMEOUT,
    I2C_MANAGER_BUSY,
    I2C_MANAGER_INVALID_PARAM
} i2c_manager_status_t;

//Function  prototypes
i2c_manager_status_t i2c_manager_init(int sda_pin, int scl_pin);
i2c_manager_status_t i2c_manager_deinit(void);

//Basic I2c operations
i2c_manager_status_t i2c_manager_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
i2c_manager_status_t i2c_manager_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
i2c_manager_status_t i2c_manager_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);
i2c_manager_status_t i2c_manager_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

//Advanced I2C operations
i2c_manager_status_t i2c_manager_write_bits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);
i2c_manager_status_t i2c_manager_read_bits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t *data);

//Utility functions
void i2c_manager_scan(void);
void i2c_manager_device_ready(uint8_t dev_addr);
const char *i2c_manager_status_string(i2c_manager_status_t status);

#endif   // I2C_MANAGER_H