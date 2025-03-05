// src/drivers/battery_monitor.c
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/adc.h>
#include <driver/gpio.h>
#include <esp_adc_cal.h>
#include <esp_log.h>

#include "drivers/battery_monitor.h"
#include "config.h"

#define TAG "BATTERY_MONITOR"

// Internal battery monitoring context
typedef struct {
    battery_monitor_config_t hardware_config;
    battery_config_t battery_config;
    battery_measurement_t last_measurement;
    esp_err_t last_error;
    
    // Calibration data
    struct {
        float voltage_offset;
        float current_offset;
        float temperature_offset[BATTERY_TEMPERATURE_CHANNELS];
    } calibration;

    // Runtime tracking
    struct {
        float total_energy_consumed;
        uint32_t last_measurement_time;
    } energy_tracking;
} battery_monitor_context_t;

// Static instance of battery monitor context
static battery_monitor_context_t battery_ctx = {0};

// ADC Calibration
static esp_adc_cal_characteristics_t adc_chars;

// Internal helper functions
static esp_err_t read_voltage(float *voltage);
static esp_err_t read_current(float *current);
static esp_err_t read_temperature(float *temperatures);
static battery_status_t evaluate_battery_status(const battery_measurement_t *measurement);
static float calculate_state_of_charge(float total_voltage);

esp_err_t battery_monitor_init(
    const battery_monitor_config_t *config,
    const battery_config_t *battery_config
) {
    if (config == NULL || battery_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Validate input pins
    if (!GPIO_IS_VALID_GPIO(config->voltage_sense_pin) ||
        !GPIO_IS_VALID_GPIO(config->current_sense_pin)) {
        ESP_LOGE(TAG, "Invalid GPIO pins for battery monitoring");
        return ESP_ERR_INVALID_ARG;
    }

    // Configure ADC for voltage and current sensing
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // Configure voltage sense pin
    adc1_config_channel_atten(
        ADC1_CHANNEL_0,  // Assuming voltage on ADC1 Channel 0
        ADC_ATTEN_DB_11  // 0-3.9V range
    );

    // Configure current sense pin
    adc1_config_channel_atten(
        ADC1_CHANNEL_1,  // Assuming current on ADC1 Channel 1
        ADC_ATTEN_DB_11
    );

    // Configure temperature sense pins
    for (int i = 0; i < BATTERY_TEMPERATURE_CHANNELS; i++) {
        if (config->temperature_pins[i] != GPIO_NUM_NC) {
            adc1_config_channel_atten(
                ADC1_CHANNEL_2 + i,  // Assuming subsequent channels
                ADC_ATTEN_DB_11
            );
        }
    }

    // Characterize ADC
    esp_adc_cal_characterize(
        ADC_UNIT_1, 
        ADC_ATTEN_DB_11, 
        ADC_WIDTH_BIT_12, 
        0,  // Reference voltage (use default)
        &adc_chars
    );

    // Store configurations
    memcpy(&battery_ctx.hardware_config, config, sizeof(battery_monitor_config_t));
    memcpy(&battery_ctx.battery_config, battery_config, sizeof(battery_config_t));

    // Initialize energy tracking
    battery_ctx.energy_tracking.last_measurement_time = xTaskGetTickCount();

    ESP_LOGI(TAG, "Battery monitor initialized");
    return ESP_OK;
}

esp_err_t battery_monitor_read(battery_measurement_t *measurement) {
    if (measurement == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read voltage
    esp_err_t ret = read_voltage(&measurement->total_voltage);
    if (ret != ESP_OK) {
        return ret;
    }

    // Read current
    ret = read_current(&measurement->current);
    if (ret != ESP_OK) {
        return ret;
    }

    // Read temperatures
    ret = read_temperature(measurement->temperature);
    if (ret != ESP_OK) {
        return ret;
    }

    // Calculate cell voltages
    for (uint8_t i = 0; i < battery_ctx.battery_config.cell_count; i++) {
        measurement->cell_voltages[i] = 
            measurement->total_voltage / battery_ctx.battery_config.cell_count;
    }

    // Calculate state of charge
    measurement->state_of_charge = 
        calculate_state_of_charge(measurement->total_voltage);

    // Estimate remaining capacity
    measurement->capacity_remaining = 
        measurement->state_of_charge * battery_ctx.battery_config.full_charge_voltage;

    // Determine battery status
    measurement->status = evaluate_battery_status(measurement);

    // Update energy tracking
    uint32_t current_time = xTaskGetTickCount();
    float time_delta = (current_time - battery_ctx.energy_tracking.last_measurement_time) 
                       / (float)configTICK_RATE_HZ;
    
    battery_ctx.energy_tracking.total_energy_consumed += 
        measurement->current * measurement->total_voltage * time_delta;
    
    battery_ctx.energy_tracking.last_measurement_time = current_time;

    // Store last measurement
    memcpy(&battery_ctx.last_measurement, measurement, sizeof(battery_measurement_t));

    return ESP_OK;
}

// Read voltage from ADC
static esp_err_t read_voltage(float *voltage) {
    uint32_t raw_voltage = adc1_get_raw(ADC1_CHANNEL_0);
    
    // Convert raw ADC to voltage using calibration
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(raw_voltage, &adc_chars);
    
    // Apply voltage divider and calibration offset
    *voltage = (voltage_mv / 1000.0f) * 
               battery_ctx.hardware_config.voltage_divider_ratio + 
               battery_ctx.calibration.voltage_offset;

    return ESP_OK;
}

// Read current from ADC
static esp_err_t read_current(float *current) {
    uint32_t raw_current = adc1_get_raw(ADC1_CHANNEL_1);
    
    // Convert raw ADC to current
    uint32_t current_mv = esp_adc_cal_raw_to_voltage(raw_current, &adc_chars);
    
    // Apply current sense gain and calibration offset
    *current = (current_mv / 1000.0f) * 
               battery_ctx.hardware_config.current_sense_gain + 
               battery_ctx.calibration.current_offset;

    return ESP_OK;
}

// Read temperatures from ADC
static esp_err_t read_temperature(float *temperatures) {
    for (int i = 0; i < BATTERY_TEMPERATURE_CHANNELS; i++) {
        if (battery_ctx.hardware_config.temperature_pins[i] != GPIO_NUM_NC) {
            uint32_t raw_temp = adc1_get_raw(ADC1_CHANNEL_2 + i);
            
            // Convert raw ADC to temperature (example conversion)
            // This is a simplified conversion and should be calibrated
            temperatures[i] = (raw_temp * 0.1f) + 
                              battery_ctx.calibration.temperature_offset[i];
        } else {
            temperatures[i] = 0.0f;
        }
    }

    return ESP_OK;
}

// Evaluate battery status based on measurements
static battery_status_t evaluate_battery_status(const battery_measurement_t *measurement) {
    // Check voltage
    if (measurement->total_voltage <= 
        battery_ctx.battery_config.critical_voltage_threshold) {
        return BATTERY_STATUS_CRITICAL;
    }

    if (measurement->total_voltage <= 
        battery_ctx.battery_config.low_voltage_threshold) {
        return BATTERY_STATUS_LOW_VOLTAGE;
    }

    // Check temperature
    for (int i = 0; i < BATTERY_TEMPERATURE_CHANNELS; i++) {
        if (measurement->temperature[i] > 60.0f) {  // 60°C threshold
            return BATTERY_STATUS_OVER_TEMPERATURE;
        }
        if (measurement->temperature[i] < -20.0f) {  // -20°C threshold
            return BATTERY_STATUS_UNDER_TEMPERATURE;
        }
    }

    return BATTERY_STATUS_NORMAL;
}

// Calculate State of Charge (SoC)
static float calculate_state_of_charge(float total_voltage) {
    const battery_config_t *config = &battery_ctx.battery_config;
    
    // Linear interpolation between min and max voltages
    float normalized = (total_voltage - (config->cell_count * config->cell_min_voltage)) /
                       ((config->cell_count * config->cell_max_voltage) - 
                        (config->cell_count * config->cell_min_voltage));
    
    // Clamp between 0 and 100
    return fmaxf(0.0f, fminf(100.0f, normalized * 100.0f));
}

esp_err_t battery_monitor_calibrate(void) {
    battery_measurement_t calibration_data;
    
    // Take multiple samples for calibration
    for (int i = 0; i < 100; i++) {
        esp_err_t ret = battery_monitor_read(&calibration_data);
        if (ret != ESP_OK) {
            return ret;
        }

        // Accumulate calibration offsets
        battery_ctx.calibration.voltage_offset += 
            calibration_data.total_voltage;
        battery_ctx.calibration.current_offset += 
            calibration_data.current;
        
        for (int j = 0; j < BATTERY_TEMPERATURE_CHANNELS; j++) {
            battery_ctx.calibration.temperature_offset[j] += 
                calibration_data.temperature[j];
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Average the offsets
    battery_ctx.calibration.voltage_offset /= 100.0f;
    battery_ctx.calibration.current_offset /= 100.0f;
    
    // Average temperature offsets
    for (int j = 0; j < BATTERY_TEMPERATURE_CHANNELS; j++) {
        battery_ctx.calibration.temperature_offset[j] /= 100.0f;
    }

    ESP_LOGI(TAG, "Battery monitor calibration complete");
    return ESP_OK;
}

bool battery_monitor_is_safe(void) {
    // Check the last measurement's status
    battery_status_t status = battery_ctx.last_measurement.status;
    
    return (status == BATTERY_STATUS_NORMAL);
}

float battery_monitor_get_estimated_runtime(float current_consumption) {
    // Calculate remaining capacity
    float remaining_capacity = battery_ctx.last_measurement.capacity_remaining;
    
    // Prevent division by zero
    if (current_consumption <= 0) {
        return INFINITY;
    }

    // Estimate runtime in minutes
    // Assuming current_consumption is in Watts
    float estimated_runtime = (remaining_capacity / current_consumption) * 60.0f;
    
    return fmaxf(0.0f, estimated_runtime);
}

esp_err_t battery_monitor_set_voltage_thresholds(
    float low_threshold, 
    float critical_threshold
) {
    // Validate thresholds
    if (low_threshold <= 0 || critical_threshold <= 0 ||
        critical_threshold >= low_threshold) {
        ESP_LOGE(TAG, "Invalid voltage thresholds");
        return ESP_ERR_INVALID_ARG;
    }

    // Update battery configuration
    battery_ctx.battery_config.low_voltage_threshold = low_threshold;
    battery_ctx.battery_config.critical_voltage_threshold = critical_threshold;

    ESP_LOGI(TAG, "Battery voltage thresholds updated: Low=%.2fV, Critical=%.2fV", 
             low_threshold, critical_threshold);

    return ESP_OK;
}

esp_err_t battery_monitor_get_last_error(void) {
    return battery_ctx.last_error;
}

// Additional diagnostic and advanced monitoring functions

/**
 * @brief Perform comprehensive battery health assessment
 * @param health_report Pointer to store detailed health report
 * @return ESP_OK on success, error code on failure
 */
esp_err_t battery_monitor_get_health_report(struct {
    float internal_resistance;
    float cycle_count;
    float max_capacity;
    bool is_balanced;
    float cell_deviation;
} *health_report) {
    if (health_report == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if we have a valid last measurement
    if (battery_ctx.last_measurement.total_voltage == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    // Calculate cell voltage deviation
    float min_cell_voltage = INFINITY;
    float max_cell_voltage = -INFINITY;
    float total_cell_voltage = 0;

    for (uint8_t i = 0; i < battery_ctx.battery_config.cell_count; i++) {
        float cell_voltage = battery_ctx.last_measurement.cell_voltages[i];
        
        min_cell_voltage = fminf(min_cell_voltage, cell_voltage);
        max_cell_voltage = fmaxf(max_cell_voltage, cell_voltage);
        total_cell_voltage += cell_voltage;
    }

    // Cell balance assessment
    health_report->cell_deviation = 
        (max_cell_voltage - min_cell_voltage) / 
        (total_cell_voltage / battery_ctx.battery_config.cell_count);
    
    health_report->is_balanced = health_report->cell_deviation < 0.05f; // 5% deviation

    // Simplified internal resistance estimation
    // This is a very basic estimation and should be refined with more sophisticated methods
    float internal_resistance = 
        (battery_ctx.battery_config.full_charge_voltage - 
         battery_ctx.last_measurement.total_voltage) / 
        battery_ctx.last_measurement.current;
    
    health_report->internal_resistance = fabsf(internal_resistance);

    // Total energy consumed can be used as a proxy for cycle count
    health_report->cycle_count = 
        battery_ctx.energy_tracking.total_energy_consumed / 
        battery_ctx.battery_config.full_charge_voltage;

    // Maximum capacity estimation
    health_report->max_capacity = 
        battery_ctx.battery_config.full_charge_voltage * 
        battery_ctx.battery_config.cell_count;

    return ESP_OK;
}

/**
 * @brief Log detailed battery information
 * @return ESP_OK on success, error code on failure
 */
esp_err_t battery_monitor_log_detailed_info(void) {
    battery_measurement_t *m = &battery_ctx.last_measurement;
    
    ESP_LOGI(TAG, "Battery Detailed Information:");
    ESP_LOGI(TAG, "Total Voltage: %.2fV", m->total_voltage);
    ESP_LOGI(TAG, "Current: %.2fA", m->current);
    ESP_LOGI(TAG, "State of Charge: %.2f%%", m->state_of_charge);
    ESP_LOGI(TAG, "Capacity Remaining: %.2fmAh", m->capacity_remaining);
    
    // Log cell voltages
    for (uint8_t i = 0; i < battery_ctx.battery_config.cell_count; i++) {
        ESP_LOGI(TAG, "Cell %d Voltage: %.2fV", i+1, m->cell_voltages[i]);
    }
    
    // Log temperatures
    for (int i = 0; i < BATTERY_TEMPERATURE_CHANNELS; i++) {
        ESP_LOGI(TAG, "Temperature %d: %.2f°C", i+1, m->temperature[i]);
    }
    
    // Log status
    switch (m->status) {
        case BATTERY_STATUS_NORMAL:
            ESP_LOGI(TAG, "Status: Normal");
            break;
        case BATTERY_STATUS_LOW_VOLTAGE:
            ESP_LOGI(TAG, "Status: Low Voltage WARNING");
            break;
        case BATTERY_STATUS_CRITICAL:
            ESP_LOGI(TAG, "Status: CRITICAL Voltage");
            break;
        case BATTERY_STATUS_OVER_TEMPERATURE:
            ESP_LOGI(TAG, "Status: OVER Temperature");
            break;
        case BATTERY_STATUS_UNDER_TEMPERATURE:
            ESP_LOGI(TAG, "Status: UNDER Temperature");
            break;
        default:
            ESP_LOGI(TAG, "Status: Unknown");
    }

    return ESP_OK;
}