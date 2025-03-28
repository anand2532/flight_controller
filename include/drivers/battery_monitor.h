#ifndef BATTERY_MONITOR_DRIVER_H
#define BATTERY_MONITOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

// Battery configuration constants
#define BATTERY_CELL_COUNT_MAX     6
#define BATTERY_TEMPERATURE_CHANNELS 2

// Battery health status
typedef enum {
    BATTERY_STATUS_NORMAL,
    BATTERY_STATUS_LOW_VOLTAGE,
    BATTERY_STATUS_CRITICAL,
    BATTERY_STATUS_OVER_TEMPERATURE,
    BATTERY_STATUS_UNDER_TEMPERATURE,
    BATTERY_STATUS_CHARGING
} battery_status_t;

// Battery chemistry types
typedef enum {
    BATTERY_CHEMISTRY_LIPO,
    BATTERY_CHEMISTRY_LION,
    BATTERY_CHEMISTRY_NIMH,
    BATTERY_CHEMISTRY_CUSTOM
} battery_chemistry_t;

// Battery configuration structure
typedef struct {
    battery_chemistry_t chemistry;
    uint8_t cell_count;
    float cell_nominal_voltage;
    float cell_min_voltage;
    float cell_max_voltage;
    float full_charge_voltage;
    float low_voltage_threshold;
    float critical_voltage_threshold;
} battery_config_t;

// Battery measurement structure
typedef struct {
    float total_voltage;             // Total battery voltage
    float cell_voltages[BATTERY_CELL_COUNT_MAX];  // Individual cell voltages
    float current;                   // Battery current (A)
    float temperature[BATTERY_TEMPERATURE_CHANNELS];  // Battery temperature(s)
    float capacity_remaining;         // Remaining capacity (mAh)
    float state_of_charge;            // State of charge (0-100%)
    battery_status_t status;          // Current battery status
    bool is_charging;                 // Charging state
} battery_measurement_t;

// Battery monitoring configuration
typedef struct {
    uint8_t voltage_sense_pin;       // GPIO pin for voltage sensing
    uint8_t current_sense_pin;       // GPIO pin for current sensing
    uint8_t temperature_pins[BATTERY_TEMPERATURE_CHANNELS];  // Temperature sense pins
    float voltage_divider_ratio;     // Voltage divider configuration
    float current_sense_gain;        // Current sensor gain
} battery_monitor_config_t;

/**
 * @brief Initialize battery monitoring system
 * @param config Battery monitor configuration
 * @param battery_config Battery chemistry and characteristics
 * @return ESP_OK on success, error code on failure
 */
esp_err_t battery_monitor_init(
    const battery_monitor_config_t *config,
    const battery_config_t *battery_config
);

/**
 * @brief Read current battery measurements
 * @param measurement Pointer to store battery measurements
 * @return ESP_OK on success, error code on failure
 */
esp_err_t battery_monitor_read(battery_measurement_t *measurement);

/**
 * @brief Calibrate battery monitoring system
 * @return ESP_OK on success, error code on failure
 */
esp_err_t battery_monitor_calibrate(void);

/**
 * @brief Check if battery is safe for continued operation
 * @return true if battery is safe, false otherwise
 */
bool battery_monitor_is_safe(void);

/**
 * @brief Get estimated time remaining at current consumption
 * @param current_consumption Current system power consumption (W)
 * @return Estimated time remaining in minutes
 */
float battery_monitor_get_estimated_runtime(float current_consumption);

/**
 * @brief Set custom low voltage and critical voltage thresholds
 * @param low_threshold Low voltage threshold (V)
 * @param critical_threshold Critical voltage threshold (V)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t battery_monitor_set_voltage_thresholds(
    float low_threshold, 
    float critical_threshold
);

/**
 * @brief Get the last error encountered by the battery monitor
 * @return Last error code
 */
esp_err_t battery_monitor_get_last_error(void);

#endif // BATTERY_MONITOR_DRIVER_H