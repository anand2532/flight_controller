// src/drivers/mpu6050.c
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_log.h>

#include "drivers/mpu6050.h"
#include "config.h"

#define TAG "MPU6050"

// Scaling factors for different configurations
static const float ACCEL_SCALE_FACTORS[] = {
    16384.0f,  // ±2g
    8192.0f,   // ±4g
    4096.0f,   // ±8g
    2048.0f    // ±16g
};

static const float GYRO_SCALE_FACTORS[] = {
    131.0f,    // ±250 degrees/sec
    65.5f,     // ±500 degrees/sec
    32.8f,     // ±1000 degrees/sec
    16.4f      // ±2000 degrees/sec
};

// Current configuration and calibration
static mpu6050_config_t current_config = {
    .gyro_scale = MPU6050_GYRO_SCALE_250DPS,
    .accel_scale = MPU6050_ACCEL_SCALE_2G,
    .low_pass_filter_enabled = true,
    .low_pass_filter_level = 3
};

static mpu6050_calibration_t calibration_data = {0};
static esp_err_t last_error = ESP_OK;
static bool is_calibrated = false;

// I2C master initialization
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Write to MPU6050 register
static esp_err_t mpu6050_write_reg(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    esp_err_t ret = i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    ret |= i2c_master_write_byte(cmd, reg_addr, true);
    ret |= i2c_master_write_byte(cmd, data, true);
    ret |= i2c_master_stop(cmd);
    
    ret |= i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// Read from MPU6050 register
static esp_err_t mpu6050_read_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    esp_err_t ret = i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    ret |= i2c_master_write_byte(cmd, reg_addr, true);
    
    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    ret |= i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    ret |= i2c_master_stop(cmd);
    
    ret |= i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// Public Functions Implementation

esp_err_t mpu6050_init(void) {
    // Initialize I2C
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        last_error = ret;
        ESP_LOGE(TAG, "I2C initialization failed");
        return ret;
    }

    // Check device connection
    if (!mpu6050_is_connected()) {
        last_error = ESP_ERR_NOT_FOUND;
        ESP_LOGE(TAG, "MPU6050 not found");
        return ESP_ERR_NOT_FOUND;
    }

    // Wake up the device and set clock source
    ret = mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) {
        last_error = ret;
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }

    // Configure low-pass filter and sample rate
    ret = mpu6050_configure_low_pass_filter(true, 3);
    if (ret != ESP_OK) {
        last_error = ret;
        ESP_LOGE(TAG, "Failed to configure low-pass filter");
        return ret;
    }

    // Set default gyro and accel scales
    ret = mpu6050_set_gyro_scale(MPU6050_GYRO_SCALE_250DPS);
    ret |= mpu6050_set_accel_scale(MPU6050_ACCEL_SCALE_2G);

    if (ret != ESP_OK) {
        last_error = ret;
        ESP_LOGE(TAG, "Failed to set sensor scales");
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6050_init_custom(const mpu6050_config_t *config) {
    esp_err_t ret = mpu6050_init();
    if (ret != ESP_OK) return ret;

    // Apply custom configuration
    ret = mpu6050_set_gyro_scale(config->gyro_scale);
    ret |= mpu6050_set_accel_scale(config->accel_scale);
    ret |= mpu6050_configure_low_pass_filter(
        config->low_pass_filter_enabled, 
        config->low_pass_filter_level
    );

    if (ret != ESP_OK) {
        last_error = ret;
        ESP_LOGE(TAG, "Failed to apply custom configuration");
        return ret;
    }

    // Update current configuration
    memcpy(&current_config, config, sizeof(mpu6050_config_t));

    return ESP_OK;
}

esp_err_t mpu6050_set_gyro_scale(mpu6050_gyro_scale_t scale) {
    esp_err_t ret = mpu6050_write_reg(MPU6050_GYRO_CONFIG, scale);
    if (ret == ESP_OK) {
        current_config.gyro_scale = scale;
    } else {
        last_error = ret;
        ESP_LOGE(TAG, "Failed to set gyro scale");
    }
    return ret;
}

esp_err_t mpu6050_set_accel_scale(mpu6050_accel_scale_t scale) {
    esp_err_t ret = mpu6050_write_reg(MPU6050_ACCEL_CONFIG, scale);
    if (ret == ESP_OK) {
        current_config.accel_scale = scale;
    } else {
        last_error = ret;
        ESP_LOGE(TAG, "Failed to set accelerometer scale");
    }
    return ret;
}

esp_err_t mpu6050_read_raw_data(mpu6050_data_t *data) {
    uint8_t buffer[14];
    esp_err_t ret = mpu6050_read_reg(MPU6050_ACCEL_XOUT_H, buffer, sizeof(buffer));
    
    if (ret != ESP_OK) {
        last_error = ret;
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }

    // Convert raw bytes to 16-bit values (big-endian)
    data->raw.accel_x = (buffer[0] << 8) | buffer[1];
    data->raw.accel_y = (buffer[2] << 8) | buffer[3];
    data->raw.accel_z = (buffer[4] << 8) | buffer[5];
    
    data->raw.temp = (buffer[6] << 8) | buffer[7];
    
    data->raw.gyro_x = (buffer[8] << 8) | buffer[9];
    data->raw.gyro_y = (buffer[10] << 8) | buffer[11];
    data->raw.gyro_z = (buffer[12] << 8) | buffer[13];

    return ESP_OK;
}

esp_err_t mpu6050_read_scaled_data(mpu6050_data_t *data) {
    esp_err_t ret = mpu6050_read_raw_data(data);
    if (ret != ESP_OK) return ret;

    // Get current scale indices
    int accel_scale_idx = current_config.accel_scale >> 3;
    int gyro_scale_idx = current_config.gyro_scale >> 3;

    // Scale accelerometer data to m/s^2
    float accel_scale = 9.81f / ACCEL_SCALE_FACTORS[accel_scale_idx];
    data->scaled.accel_x = data->raw.accel_x * accel_scale;
    data->scaled.accel_y = data->raw.accel_y * accel_scale;
    data->scaled.accel_z = data->raw.accel_z * accel_scale;

    // Scale gyroscope data to degrees/sec
    float gyro_scale = 1.0f / GYRO_SCALE_FACTORS[gyro_scale_idx];
    data->scaled.gyro_x = data->raw.gyro_x * gyro_scale;
    data->scaled.gyro_y = data->raw.gyro_y * gyro_scale;
    data->scaled.gyro_z = data->raw.gyro_z * gyro_scale;

    // Convert temperature to Celsius
    data->scaled.temperature = (data->raw.temp / 340.0f) + 36.53f;

    // Apply calibration if available
    if (is_calibrated) {
        data->scaled.accel_x -= calibration_data.accel_offset_x;
        data->scaled.accel_y -= calibration_data.accel_offset_y;
        data->scaled.accel_z -= calibration_data.accel_offset_z;
        
        data->scaled.gyro_x -= calibration_data.gyro_offset_x;
        data->scaled.gyro_y -= calibration_data.gyro_offset_y;
        data->scaled.gyro_z -= calibration_data.gyro_offset_z;
    }

    return ESP_OK;
}

esp_err_t mpu6050_calibrate(mpu6050_calibration_t *cal_data, uint16_t num_samples) {
    mpu6050_data_t data;
    mpu6050_calibration_t offsets = {0};

    // Reset calibration data
    memset(cal_data, 0, sizeof(mpu6050_calibration_t));

    // Collect samples
    for (uint16_t i = 0; i < num_samples; i++) {
        esp_err_t ret = mpu6050_read_scaled_data(&data);
        if (ret != ESP_OK) {
            last_error = ret;
            return ret;
        }

        // Accumulate readings
        offsets.accel_offset_x += data.scaled.accel_x;
        offsets.accel_offset_y += data.scaled.accel_y;
        offsets.accel_offset_z += data.scaled.accel_z;
        
        offsets.gyro_offset_x += data.scaled.gyro_x;
        offsets.gyro_offset_y += data.scaled.gyro_y;
        offsets.gyro_offset_z += data.scaled.gyro_z;

        // Short delay between samples
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Calculate average offsets
    cal_data->accel_offset_x = offsets.accel_offset_x / num_samples;
    cal_data->accel_offset_y = offsets.accel_offset_y / num_samples;
    
    // Adjust Z-axis to account for gravity
    cal_data->accel_offset_z = offsets.accel_offset_z / num_samples - 9.81f;
    
    cal_data->gyro_offset_x = offsets.gyro_offset_x / num_samples;
    cal_data->gyro_offset_y = offsets.gyro_offset_y / num_samples;
    cal_data->gyro_offset_z = offsets.gyro_offset_z / num_samples;

    return ESP_OK;
}

esp_err_t mpu6050_apply_calibration(const mpu6050_calibration_t *cal_data) {
    // Copy calibration data
    memcpy(&calibration_data, cal_data, sizeof(mpu6050_calibration_t));
    is_calibrated = true;

    return ESP_OK;
}

bool mpu6050_is_connected(void) {
    uint8_t data;
    esp_err_t ret = mpu6050_read_reg(MPU6050_PWR_MGMT_1, &data, 1);
    return (ret == ESP_OK);
}

esp_err_t mpu6050_get_last_error(void) {
    return last_error;
}

esp_err_t mpu6050_configure_low_pass_filter(bool enable, uint8_t filter_level) {
    if (filter_level > 6) {
        ESP_LOGE(TAG, "Invalid low-pass filter level. Must be 0-6.");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t config_value = enable ? (filter_level & 0x07) : 0;
    esp_err_t ret = mpu6050_write_reg(MPU6050_CONFIG, config_value);

    if (ret == ESP_OK) {
        current_config.low_pass_filter_enabled = enable;
        current_config.low_pass_filter_level = filter_level;
    } else {
        last_error = ret;
        ESP_LOGE(TAG, "Failed to configure low-pass filter");
    }

    return ret;
}

// Advanced diagnostic function to get detailed sensor health
esp_err_t mpu6050_diagnostic_check(void) {
    uint8_t who_am_i;
    esp_err_t ret;

    // Check device connection and WHO_AM_I register
    ret = mpu6050_read_reg(0x75, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }

    // Verify expected value (typically 0x68 for MPU6050)
    if (who_am_i != 0x68) {
        ESP_LOGE(TAG, "Incorrect device ID: 0x%02X (expected 0x68)", who_am_i);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Perform a self-test (optional but recommended)
    mpu6050_data_t test_data;
    ret = mpu6050_read_scaled_data(&test_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data during diagnostic");
        return ret;
    }

    // Validate sensor readings are within reasonable ranges
    bool data_valid = true;
    if (fabsf(test_data.scaled.accel_x) > 20.0f ||
        fabsf(test_data.scaled.accel_y) > 20.0f ||
        fabsf(test_data.scaled.accel_z) > 20.0f) {
        ESP_LOGW(TAG, "Unusual accelerometer readings");
        data_valid = false;
    }

    if (fabsf(test_data.scaled.gyro_x) > 500.0f ||
        fabsf(test_data.scaled.gyro_y) > 500.0f ||
        fabsf(test_data.scaled.gyro_z) > 500.0f) {
        ESP_LOGW(TAG, "Unusual gyroscope readings");
        data_valid = false;
    }

    if (!data_valid) {
        ESP_LOGE(TAG, "Sensor data out of expected range");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "MPU6050 diagnostic check passed");
    return ESP_OK;
}

// Function to compute and return sensor orientation
esp_err_t mpu6050_compute_orientation(float *pitch, float *roll) {
    mpu6050_data_t sensor_data;
    esp_err_t ret = mpu6050_read_scaled_data(&sensor_data);
    if (ret != ESP_OK) {
        return ret;
    }

    // Compute pitch and roll using accelerometer data
    *pitch = atan2f(sensor_data.scaled.accel_y, 
                    sqrtf(sensor_data.scaled.accel_x * sensor_data.scaled.accel_x + 
                          sensor_data.scaled.accel_z * sensor_data.scaled.accel_z)) 
             * 180.0f / M_PI;
    
    *roll = atan2f(-sensor_data.scaled.accel_x, 
                   sensor_data.scaled.accel_z) 
            * 180.0f / M_PI;

    return ESP_OK;
}

// Power management functions
esp_err_t mpu6050_set_sleep_mode(bool enable) {
    // Read current power management register
    uint8_t pwr_mgmt;
    esp_err_t ret = mpu6050_read_reg(MPU6050_PWR_MGMT_1, &pwr_mgmt, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    // Modify sleep bit
    if (enable) {
        pwr_mgmt |= 0x40;  // Set sleep bit
    } else {
        pwr_mgmt &= ~0x40; // Clear sleep bit
    }

    // Write back to register
    ret = mpu6050_write_reg(MPU6050_PWR_MGMT_1, pwr_mgmt);
    if (ret != ESP_OK) {
        last_error = ret;
        ESP_LOGE(TAG, "Failed to %s sleep mode", enable ? "enable" : "disable");
    }

    return ret;
}

// Interrupt configuration
esp_err_t mpu6050_configure_interrupt(bool enable, uint8_t interrupt_config) {
    esp_err_t ret;

    // Configure interrupt pin
    ret = mpu6050_write_reg(MPU6050_INT_PIN_CFG, interrupt_config);
    if (ret != ESP_OK) {
        last_error = ret;
        ESP_LOGE(TAG, "Failed to configure interrupt pin");
        return ret;
    }

    // Enable or disable interrupts
    ret = mpu6050_write_reg(MPU6050_INT_ENABLE, enable ? 0x01 : 0x00);
    if (ret != ESP_OK) {
        last_error = ret;
        ESP_LOGE(TAG, "Failed to %s interrupts", enable ? "enable" : "disable");
    }

    return ret;
}

// Comprehensive sensor reset
esp_err_t mpu6050_reset(void) {
    // Reset entire device
    esp_err_t ret = mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) {
        last_error = ret;
        ESP_LOGE(TAG, "Device reset failed");
        return ret;
    }

    // Short delay to allow reset to complete
    vTaskDelay(pdMS_TO_TICKS(100));

    // Reinitialize with default configuration
    return mpu6050_init();
}

// Getter for current configuration
esp_err_t mpu6050_get_current_config(mpu6050_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(config, &current_config, sizeof(mpu6050_config_t));
    return ESP_OK;
}