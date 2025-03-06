// src/utils/calibration.c
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include "utils/calibration.h"
#include "drivers/mpu6050.h"
#include "drivers/motor_control.h"
#include "drivers/radio_receiver.h"
#include "utils/logging.h"

#define TAG "QUADCOPTER_CALIBRATION"

// Calibration context
typedef struct {
    calibration_type_t current_type;
    calibration_status_t current_status;
    calibration_result_t current_result;
    bool is_calibrating;
    uint32_t start_time;
    nvs_handle_t nvs_handle;
} calibration_context_t;

// Static calibration context
static calibration_context_t cal_ctx = {0};

// NVS storage keys
#define NVS_NAMESPACE "quadcopter_cal"
#define NVS_ACCEL_KEY "accel_cal"
#define NVS_GYRO_KEY "gyro_cal"
#define NVS_ESC_KEY "esc_cal"
#define NVS_RECEIVER_KEY "rx_cal"

// Internal helper functions
static esp_err_t calibrate_accelerometer(calibration_result_t *result);
static esp_err_t calibrate_gyroscope(calibration_result_t *result);
static esp_err_t calibrate_esc(calibration_result_t *result);
static esp_err_t calibrate_radio_receiver(calibration_result_t *result);

esp_err_t calibration_init(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Erase and retry if needed
        ret = nvs_flash_erase();
        ret |= nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed");
        return ret;
    }

    // Open NVS namespace
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &cal_ctx.nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
        return ret;
    }

    // Initialize calibration context
    memset(&cal_ctx, 0, sizeof(calibration_context_t));
    cal_ctx.current_status = CALIBRATION_STATUS_NOT_STARTED;

    log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, "Calibration system initialized");
    return ESP_OK;
}

esp_err_t calibration_start(
    const calibration_config_t *config,
    calibration_result_t *result
) {
    if (!config || !result) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if a calibration is already in progress
    if (cal_ctx.is_calibrating) {
        log_message(LOG_LEVEL_WARNING, LOG_TYPE_SYSTEM, 
                    "Calibration already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Reset result
    memset(result, 0, sizeof(calibration_result_t));
    result->type = config->type;
    result->status = CALIBRATION_STATUS_IN_PROGRESS;

    // Set context
    cal_ctx.current_type = config->type;
    cal_ctx.is_calibrating = true;
    cal_ctx.start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Perform calibration based on type
    esp_err_t ret;
    switch (config->type) {
        case CALIBRATION_TYPE_ACCELEROMETER:
            ret = calibrate_accelerometer(result);
            break;
        case CALIBRATION_TYPE_GYROSCOPE:
            ret = calibrate_gyroscope(result);
            break;
        case CALIBRATION_TYPE_ESC:
            ret = calibrate_esc(result);
            break;
        case CALIBRATION_TYPE_RADIO_RECEIVER:
            ret = calibrate_radio_receiver(result);
            break;
        default:
            log_message(LOG_LEVEL_ERROR, LOG_TYPE_SYSTEM, 
                        "Unsupported calibration type");
            ret = ESP_ERR_NOT_SUPPORTED;
    }

    // Update status
    if (ret == ESP_OK) {
        result->status = CALIBRATION_STATUS_SUCCESS;
        log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                    "Calibration for type %d completed successfully", config->type);
    } else {
        result->status = CALIBRATION_STATUS_FAILED;
        log_message(LOG_LEVEL_ERROR, LOG_TYPE_SYSTEM, 
                    "Calibration for type %d failed", config->type);
    }

    // Reset calibration context
    cal_ctx.is_calibrating = false;
    cal_ctx.current_status = result->status;

    return ret;
}

// Accelerometer Calibration
static esp_err_t calibrate_accelerometer(calibration_result_t *result) {
    // Placeholders for measurement collections
    float accel_x_min = FLT_MAX, accel_x_max = -FLT_MAX;
    float accel_y_min = FLT_MAX, accel_y_max = -FLT_MAX;
    float accel_z_min = FLT_MAX, accel_z_max = -FLT_MAX;

    // Collect samples from multiple orientations
    const int num_samples = 500;
    mpu6050_data_t sample;

    log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                "Starting accelerometer calibration. Place quadcopter in different orientations.");

    for (int i = 0; i < num_samples; i++) {
        esp_err_t ret = mpu6050_read_raw_data(&sample);
        if (ret != ESP_OK) {
            return ret;
        }

        // Update min/max for each axis
        accel_x_min = fminf(accel_x_min, sample.scaled.accel_x);
        accel_x_max = fmaxf(accel_x_max, sample.scaled.accel_x);
        
        accel_y_min = fminf(accel_y_min, sample.scaled.accel_y);
        accel_y_max = fmaxf(accel_y_max, sample.scaled.accel_y);
        
        accel_z_min = fminf(accel_z_min, sample.scaled.accel_z);
        accel_z_max = fmaxf(accel_z_max, sample.scaled.accel_z);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Compute offsets and scales
    accel_calibration_data_t *cal = &result->data.accel;
    cal->offset_x = (accel_x_min + accel_x_max) / 2.0f;
    cal->offset_y = (accel_y_min + accel_y_max) / 2.0f;
    cal->offset_z = (accel_z_min + accel_z_max) / 2.0f;

    cal->scale_x = 1.0f / ((accel_x_max - accel_x_min) / 2.0f);
    cal->scale_y = 1.0f / ((accel_y_max - accel_y_min) / 2.0f);
    cal->scale_z = 1.0f / ((accel_z_max - accel_z_min) / 2.0f);

    return ESP_OK;
}

// Gyroscope Calibration
static esp_err_t calibrate_gyroscope(calibration_result_t *result) {
    float gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
    const int num_samples = 1000;
    mpu6050_data_t sample;

    log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                "Starting gyroscope calibration. Keep quadcopter absolutely still.");

    for (int i = 0; i < num_samples; i++) {
        esp_err_t ret = mpu6050_read_raw_data(&sample);
        if (ret != ESP_OK) {
            return ret;
        }

        gyro_x_sum += sample.scaled.gyro_x;
        gyro_y_sum += sample.scaled.gyro_y;
        gyro_z_sum += sample.scaled.gyro_z;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Compute offsets
    gyro_calibration_data_t *cal = &result->data.gyro;
    cal->offset_x = gyro_x_sum / num_samples;
    cal->offset_y = gyro_y_sum / num_samples;
    cal->offset_z = gyro_z_sum / num_samples;

    return ESP_OK;
}

// ESC Calibration
static esp_err_t calibrate_esc(calibration_result_t *result) {
    log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                "Starting ESC calibration. Follow calibration procedure.");

    // Typical ESC calibration procedure
    motor_outputs_t outputs;
    
    // Set all motors to maximum pulse width
    for (int i = 0; i < 4; i++) {
        outputs.speeds[i] = 2000;
    }
    motor_control_set_speeds(&outputs);
    vTaskDelay(pdMS_TO_TICKS(5000));  // 5-second high pulse

    // Set all motors to minimum pulse width
    // Continue ESC calibration function
    for (int i = 0; i < 4; i++) {
        outputs.speeds[i] = 1000;
    }
    motor_control_set_speeds(&outputs);
    vTaskDelay(pdMS_TO_TICKS(5000));  // 5-second low pulse

    // Store calibration data
    esc_calibration_data_t *cal = &result->data.esc;
    cal->min_pulse_width = 1000;
    cal->max_pulse_width = 2000;
    cal->neutral_pulse_width = 1500;

    log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                "ESC calibration completed");
    return ESP_OK;
}

// Radio Receiver Calibration
static esp_err_t calibrate_radio_receiver(calibration_result_t *result) {
    receiver_calibration_data_t *cal = &result->data.receiver;
    receiver_input_t input;
    
    log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                "Starting radio receiver calibration. Move all sticks to extremes.");

    // Initialize min/max values
    for (int ch = 0; ch < 8; ch++) {
        cal->min_values[ch] = UINT16_MAX;
        cal->max_values[ch] = 0;
    }

    // Collect calibration data
    const int num_samples = 1000;
    for (int i = 0; i < num_samples; i++) {
        esp_err_t ret = radio_receiver_read(&input);
        if (ret != ESP_OK) {
            return ret;
        }

        // Update min/max for each channel
        for (int ch = 0; ch < 8; ch++) {
            cal->min_values[ch] = fminf(cal->min_values[ch], input.channels[ch]);
            cal->max_values[ch] = fmaxf(cal->max_values[ch], input.channels[ch]);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Compute mid values
    for (int ch = 0; ch < 8; ch++) {
        cal->mid_values[ch] = 
            (cal->min_values[ch] + cal->max_values[ch]) / 2;
    }

    log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                "Radio receiver calibration completed");
    return ESP_OK;
}

esp_err_t calibration_abort(void) {
    if (!cal_ctx.is_calibrating) {
        return ESP_ERR_INVALID_STATE;
    }

    // Reset calibration context
    cal_ctx.is_calibrating = false;
    cal_ctx.current_status = CALIBRATION_STATUS_FAILED;

    log_message(LOG_LEVEL_WARNING, LOG_TYPE_SYSTEM, 
                "Calibration aborted");

    return ESP_OK;
}

esp_err_t calibration_save(const calibration_result_t *result) {
    if (!result) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    size_t required_size;

    // Save based on calibration type
    switch (result->type) {
        case CALIBRATION_TYPE_ACCELEROMETER:
            required_size = sizeof(accel_calibration_data_t);
            ret = nvs_set_blob(cal_ctx.nvs_handle, NVS_ACCEL_KEY, 
                                &result->data.accel, required_size);
            break;

        case CALIBRATION_TYPE_GYROSCOPE:
            required_size = sizeof(gyro_calibration_data_t);
            ret = nvs_set_blob(cal_ctx.nvs_handle, NVS_GYRO_KEY, 
                                &result->data.gyro, required_size);
            break;

        case CALIBRATION_TYPE_ESC:
            required_size = sizeof(esc_calibration_data_t);
            ret = nvs_set_blob(cal_ctx.nvs_handle, NVS_ESC_KEY, 
                                &result->data.esc, required_size);
            break;

        case CALIBRATION_TYPE_RADIO_RECEIVER:
            required_size = sizeof(receiver_calibration_data_t);
            ret = nvs_set_blob(cal_ctx.nvs_handle, NVS_RECEIVER_KEY, 
                                &result->data.receiver, required_size);
            break;

        default:
            log_message(LOG_LEVEL_ERROR, LOG_TYPE_SYSTEM, 
                        "Unsupported calibration type for saving");
            return ESP_ERR_NOT_SUPPORTED;
    }

    // Commit changes
    if (ret == ESP_OK) {
        ret = nvs_commit(cal_ctx.nvs_handle);
    }

    if (ret == ESP_OK) {
        log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                    "Calibration data saved successfully");
    } else {
        log_message(LOG_LEVEL_ERROR, LOG_TYPE_SYSTEM, 
                    "Failed to save calibration data");
    }

    return ret;
}

esp_err_t calibration_load(
    calibration_type_t type, 
    calibration_result_t *result
) {
    if (!result) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    size_t required_size;

    // Clear result structure
    memset(result, 0, sizeof(calibration_result_t));
    result->type = type;

    // Load based on calibration type
    switch (type) {
        case CALIBRATION_TYPE_ACCELEROMETER:
            required_size = sizeof(accel_calibration_data_t);
            ret = nvs_get_blob(cal_ctx.nvs_handle, NVS_ACCEL_KEY, 
                                &result->data.accel, &required_size);
            break;

        case CALIBRATION_TYPE_GYROSCOPE:
            required_size = sizeof(gyro_calibration_data_t);
            ret = nvs_get_blob(cal_ctx.nvs_handle, NVS_GYRO_KEY, 
                                &result->data.gyro, &required_size);
            break;

        case CALIBRATION_TYPE_ESC:
            required_size = sizeof(esc_calibration_data_t);
            ret = nvs_get_blob(cal_ctx.nvs_handle, NVS_ESC_KEY, 
                                &result->data.esc, &required_size);
            break;

        case CALIBRATION_TYPE_RADIO_RECEIVER:
            required_size = sizeof(receiver_calibration_data_t);
            ret = nvs_get_blob(cal_ctx.nvs_handle, NVS_RECEIVER_KEY, 
                                &result->data.receiver, &required_size);
            break;

        default:
            log_message(LOG_LEVEL_ERROR, LOG_TYPE_SYSTEM, 
                        "Unsupported calibration type for loading");
            return ESP_ERR_NOT_SUPPORTED;
    }

    // Update status based on load result
    if (ret == ESP_OK) {
        result->status = CALIBRATION_STATUS_SUCCESS;
        log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                    "Calibration data loaded successfully");
    } else {
        result->status = CALIBRATION_STATUS_FAILED;
        log_message(LOG_LEVEL_WARNING, LOG_TYPE_SYSTEM, 
                    "Failed to load calibration data");
    }

    return ret;
}

esp_err_t calibration_reset(calibration_type_t type) {
    esp_err_t ret;

    // Erase specific calibration data
    switch (type) {
        case CALIBRATION_TYPE_ACCELEROMETER:
            ret = nvs_erase_key(cal_ctx.nvs_handle, NVS_ACCEL_KEY);
            break;

        case CALIBRATION_TYPE_GYROSCOPE:
            ret = nvs_erase_key(cal_ctx.nvs_handle, NVS_GYRO_KEY);
            break;

        case CALIBRATION_TYPE_ESC:
            ret = nvs_erase_key(cal_ctx.nvs_handle, NVS_ESC_KEY);
            break;

        case CALIBRATION_TYPE_RADIO_RECEIVER:
            ret = nvs_erase_key(cal_ctx.nvs_handle, NVS_RECEIVER_KEY);
            break;

        default:
            log_message(LOG_LEVEL_ERROR, LOG_TYPE_SYSTEM, 
                        "Unsupported calibration type for reset");
            return ESP_ERR_NOT_SUPPORTED;
    }

    // Commit changes
    if (ret == ESP_OK) {
        ret = nvs_commit(cal_ctx.nvs_handle);
    }

    if (ret == ESP_OK) {
        log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                    "Calibration data reset successfully");
    } else {
        log_message(LOG_LEVEL_ERROR, LOG_TYPE_SYSTEM, 
                    "Failed to reset calibration data");
    }

    return ret;
}

esp_err_t calibration_get_status(
    calibration_type_t type, 
    calibration_status_t *status
) {
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if calibration data exists
    size_t required_size;
    esp_err_t ret;

    switch (type) {
        case CALIBRATION_TYPE_ACCELEROMETER:
            ret = nvs_get_blob(cal_ctx.nvs_handle, NVS_ACCEL_KEY, NULL, &required_size);
            break;

        case CALIBRATION_TYPE_GYROSCOPE:
            ret = nvs_get_blob(cal_ctx.nvs_handle, NVS_GYRO_KEY, NULL, &required_size);
            break;

        case CALIBRATION_TYPE_ESC:
            ret = nvs_get_blob(cal_ctx.nvs_handle, NVS_ESC_KEY, NULL, &required_size);
            break;

        case CALIBRATION_TYPE_RADIO_RECEIVER:
            ret = nvs_get_blob(cal_ctx.nvs_handle, NVS_RECEIVER_KEY, NULL, &required_size);
            break;

        default:
            return ESP_ERR_NOT_SUPPORTED;
    }

    // Set status based on existence of calibration data
    *status = (ret == ESP_OK) ? 
              CALIBRATION_STATUS_SUCCESS : 
              CALIBRATION_STATUS_NOT_STARTED;

    return ESP_OK;
}

esp_err_t calibration_perform_full_system(
    calibration_result_t *result
) {
    esp_err_t ret;
    calibration_config_t config;

    // Perform comprehensive calibration
    calibration_type_t calibration_types[] = {
        CALIBRATION_TYPE_ACCELEROMETER,
        CALIBRATION_TYPE_GYROSCOPE,
        CALIBRATION_TYPE_ESC,
        CALIBRATION_TYPE_RADIO_RECEIVER
    };

    // Initialize full results
    memset(result, 0, sizeof(calibration_result_t));
    result->status = CALIBRATION_STATUS_IN_PROGRESS;

    // Perform calibration for each type
    for (size_t i = 0; i < sizeof(calibration_types)/sizeof(calibration_type_t); i++) {
        config.type = calibration_types[i];
        config.num_samples = 1000;
        config.timeout_seconds = 60.0f;

        // Perform individual calibration
        ret = calibration_start(&config, result);
        if (ret != ESP_OK) {
            log_message(LOG_LEVEL_ERROR, LOG_TYPE_SYSTEM, 
                        "Full system calibration failed for type %d", 
                        calibration_types[i]);
            result->status = CALIBRATION_STATUS_FAILED;
            return ret;
        }

        // Save each calibration result
        ret = calibration_save(result);
        if (ret != ESP_OK) {
            log_message(LOG_LEVEL_ERROR, LOG_TYPE_SYSTEM, 
                        "Failed to save calibration for type %d", 
                        calibration_types[i]);
            result->status = CALIBRATION_STATUS_FAILED;
            return ret;
        }
    }

    // Mark full calibration as successful
    result->status = CALIBRATION_STATUS_SUCCESS;
    log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                "Full system calibration completed successfully");

    return ESP_OK;
}