// include/drivers/mpu6050.h
#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c.h>

// MPU6050 Register Addresses
#define MPU6050_ADDR                    0x68
#define MPU6050_PWR_MGMT_1              0x6B
#define MPU6050_ACCEL_XOUT_H            0x3B
#define MPU6050_GYRO_XOUT_H             0x43
#define MPU6050_CONFIG                  0x1A
#define MPU6050_GYRO_CONFIG             0x1B
#define MPU6050_ACCEL_CONFIG            0x1C
#define MPU6050_USER_CTRL               0x6A
#define MPU6050_INT_PIN_CFG             0x37
#define MPU6050_INT_ENABLE              0x38

// I2C Configuration
#define I2C_MASTER_NUM                  I2C_NUM_0
#define I2C_MASTER_FREQ_HZ              400000
#define I2C_MASTER_SDA_IO               GPIO_NUM_21
#define I2C_MASTER_SCL_IO               GPIO_NUM_22

// Gyro Scale Ranges and Sensitivity
typedef enum {
    MPU6050_GYRO_SCALE_250DPS  = 0x00,   // ±250 degrees/sec
    MPU6050_GYRO_SCALE_500DPS  = 0x08,   // ±500 degrees/sec
    MPU6050_GYRO_SCALE_1000DPS = 0x10,   // ±1000 degrees/sec
    MPU6050_GYRO_SCALE_2000DPS = 0x18    // ±2000 degrees/sec
} mpu6050_gyro_scale_t;

// Accelerometer Scale Ranges and Sensitivity
typedef enum {
    MPU6050_ACCEL_SCALE_2G  = 0x00,      // ±2g
    MPU6050_ACCEL_SCALE_4G  = 0x08,      // ±4g
    MPU6050_ACCEL_SCALE_8G  = 0x10,      // ±8g
    MPU6050_ACCEL_SCALE_16G = 0x18       // ±16g
} mpu6050_accel_scale_t;

// Sensor data structure with raw and scaled data
typedef struct {
    // Raw sensor readings (16-bit integers)
    struct {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t temp;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    } raw;

    // Processed sensor data (float values)
    struct {
        float accel_x;     // Acceleration X-axis (m/s^2)
        float accel_y;     // Acceleration Y-axis (m/s^2)
        float accel_z;     // Acceleration Z-axis (m/s^2)
        float gyro_x;      // Angular velocity X-axis (degrees/sec)
        float gyro_y;      // Angular velocity Y-axis (degrees/sec)
        float gyro_z;      // Angular velocity Z-axis (degrees/sec)
        float temperature; // Temperature in degrees Celsius
    } scaled;
} mpu6050_data_t;

// Calibration data structure for sensor offsets
typedef struct {
    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
} mpu6050_calibration_t;

// Configuration structure for MPU6050
typedef struct {
    mpu6050_gyro_scale_t gyro_scale;
    mpu6050_accel_scale_t accel_scale;
    bool low_pass_filter_enabled;
    uint8_t low_pass_filter_level;
} mpu6050_config_t;

/**
 * @brief Initialize the MPU6050 sensor with default configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_init(void);

/**
 * @brief Initialize the MPU6050 with custom configuration
 * @param config Pointer to custom configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_init_custom(const mpu6050_config_t *config);

/**
 * @brief Configure the gyroscope scale
 * @param scale Gyroscope scale to set
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_set_gyro_scale(mpu6050_gyro_scale_t scale);

/**
 * @brief Configure the accelerometer scale
 * @param scale Accelerometer scale to set
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_set_accel_scale(mpu6050_accel_scale_t scale);

/**
 * @brief Read raw sensor data from MPU6050
 * @param data Pointer to store sensor data
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_read_raw_data(mpu6050_data_t *data);

/**
 * @brief Read processed sensor data (with scaling applied)
 * @param data Pointer to store processed sensor data
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_read_scaled_data(mpu6050_data_t *data);

/**
 * @brief Perform sensor calibration
 * @param calibration_data Pointer to store calibration offsets
 * @param num_samples Number of samples to use for calibration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_calibrate(mpu6050_calibration_t *calibration_data, 
                             uint16_t num_samples);

/**
 * @brief Apply calibration offsets to sensor readings
 * @param calibration_data Calibration offsets to apply
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_apply_calibration(const mpu6050_calibration_t *calibration_data);

/**
 * @brief Check if the MPU6050 is connected and responding
 * @return true if connected, false otherwise
 */
bool mpu6050_is_connected(void);

/**
 * @brief Get the last error encountered by the MPU6050 driver
 * @return Last error code
 */
esp_err_t mpu6050_get_last_error(void);

/**
 * @brief Enable or disable low-pass filter
 * @param enable Flag to enable or disable
 * @param filter_level Low-pass filter level (0-6)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mpu6050_configure_low_pass_filter(bool enable, uint8_t filter_level);

#endif // MPU6050_DRIVER_H