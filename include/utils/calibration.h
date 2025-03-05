// include/utils/calibration.h
#ifndef QUADCOPTER_CALIBRATION_H
#define QUADCOPTER_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

// Calibration types
typedef enum {
    CALIBRATION_TYPE_ACCELEROMETER,
    CALIBRATION_TYPE_GYROSCOPE,
    CALIBRATION_TYPE_MAGNETOMETER,
    CALIBRATION_TYPE_ESC,
    CALIBRATION_TYPE_RADIO_RECEIVER
} calibration_type_t;

// Calibration status
typedef enum {
    CALIBRATION_STATUS_NOT_STARTED,
    CALIBRATION_STATUS_IN_PROGRESS,
    CALIBRATION_STATUS_SUCCESS,
    CALIBRATION_STATUS_FAILED
} calibration_status_t;

// Accelerometer calibration data
typedef struct {
    float offset_x;
    float offset_y;
    float offset_z;
    float scale_x;
    float scale_y;
    float scale_z;
} accel_calibration_data_t;

// Gyroscope calibration data
typedef struct {
    float offset_x;
    float offset_y;
    float offset_z;
} gyro_calibration_data_t;

// ESC calibration data
typedef struct {
    uint16_t min_pulse_width;
    uint16_t max_pulse_width;
    uint16_t neutral_pulse_width;
} esc_calibration_data_t;

// Radio receiver calibration data
typedef struct {
    uint16_t min_values[8];
    uint16_t max_values[8];
    uint16_t mid_values[8];
} receiver_calibration_data_t;

// Calibration configuration
typedef struct {
    calibration_type_t type;
    uint16_t num_samples;
    float timeout_seconds;
} calibration_config_t;

// Calibration result structure
typedef struct {
    calibration_type_t type;
    calibration_status_t status;
    union {
        accel_calibration_data_t accel;
        gyro_calibration_data_t gyro;
        esc_calibration_data_t esc;
        receiver_calibration_data_t receiver;
    } data;
} calibration_result_t;

/**
 * @brief Initialize calibration system
 * @return ESP_OK on success, error code on failure
 */
esp_err_t calibration_init(void);

/**
 * @brief Start calibration procedure
 * @param config Calibration configuration
 * @param result Pointer to store calibration results
 * @return ESP_OK on success, error code on failure
 */
esp_err_t calibration_start(
    const calibration_config_t *config,
    calibration_result_t *result
);

/**
 * @brief Abort ongoing calibration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t calibration_abort(void);

/**
 * @brief Save calibration data to non-volatile storage
 * @param result Calibration result to save
 * @return ESP_OK on success, error code on failure
 */
esp_err_t calibration_save(const calibration_result_t *result);

/**
 * @brief Load calibration data from non-volatile storage
 * @param type Calibration type to load
 * @param result Pointer to store loaded calibration data
 * @return ESP_OK on success, error code on failure
 */
esp_err_t calibration_load(
    calibration_type_t type, 
    calibration_result_t *result
);

/**
 * @brief Reset calibration data for a specific type
 * @param type Calibration type to reset
 * @return ESP_OK on success, error code on failure
 */
esp_err_t calibration_reset(calibration_type_t type);

/**
 * @brief Get status of a specific calibration type
 * @param type Calibration type
 * @param status Pointer to store calibration status
 * @return ESP_OK on success, error code on failure
 */
esp_err_t calibration_get_status(
    calibration_type_t type, 
    calibration_status_t *status
);

/**
 * @brief Perform a comprehensive system calibration
 * @param result Pointer to store comprehensive calibration results
 * @return ESP_OK on success, error code on failure
 */
esp_err_t calibration_perform_full_system(
    calibration_result_t *result
);

#endif // QUADCOPTER_CALIBRATION_H