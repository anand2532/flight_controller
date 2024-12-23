#include "utils/i2c_manager.h"
#include "config.h"
#include "esp_log.h"
#include <cstdint>

#define I2C_MANAGER_TIMEOUT_MS 1000

static const char *TAG = "I2C_MANAGER";

esp_err_t i2c_manager_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };

    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed");
        return err; 
    }

    esp_err_t err = i2c_param_config(I2C_NUM_O, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed");
    }
    return err;
}

esp_err_t i2c_manager_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte()
}

esp_err_t i2c_manager_read_bytes() {

}

esp_err_t i2c_manager_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    return i2c_manager_write_bytes(dev_addr, reg_addr, &data, 1);
}

esp_err_t i2c_manager_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
    return i2c_manager_read_bytes(dev_addr, reg_addr, data, 1);
}