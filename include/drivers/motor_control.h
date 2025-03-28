#ifndef MOTOR_CONTROL_DRIVER_H
#define MOTOR_CONTROL_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

// Motor configuration constants
#define MAX_MOTORS                  4
#define MOTOR_MIN_PULSE_WIDTH       1000    // Minimum ESC pulse width (μs)
#define MOTOR_MAX_PULSE_WIDTH       2000    // Maximum ESC pulse width (μs)
#define MOTOR_IDLE_PULSE_WIDTH      1100    // Idle pulse width (μs)
#define MOTOR_CALIBRATION_DELAY_MS  2000    // Delay during ESC calibration

// Quadcopter frame configurations
typedef enum {
    QUADCOPTER_X_CONFIG,
    QUADCOPTER_PLUS_CONFIG,
    HEXACOPTER_CONFIG
} motor_frame_config_t;

// Motor output structure
typedef struct {
    uint16_t speeds[MAX_MOTORS];  // PWM values for each motor (1000-2000μs)
} motor_outputs_t;

// ESC calibration modes
typedef enum {
    ESC_CALIBRATION_LOW,
    ESC_CALIBRATION_HIGH,
    ESC_CALIBRATION_STANDARD
} esc_calibration_mode_t;

// Motor pin configuration
typedef struct {
    uint8_t motor_pins[MAX_MOTORS];    // GPIO pins for each motor
    motor_frame_config_t frame_type;   // Quadcopter frame configuration
    uint16_t pwm_frequency;            // PWM frequency in Hz
} motor_config_t;

// Motor health and status
typedef struct {
    bool is_armed;
    bool motor_enabled[MAX_MOTORS];
    uint16_t current_speeds[MAX_MOTORS];
    uint32_t total_running_time;       // Total motor running time
} motor_status_t;

// Error codes specific to motor control
typedef enum {
    MOTOR_ERROR_NONE = 0,
    MOTOR_ERROR_INITIALIZATION_FAILED,
    MOTOR_ERROR_INVALID_SPEED,
    MOTOR_ERROR_NOT_ARMED,
    MOTOR_ERROR_CALIBRATION_FAILED
} motor_error_t;

/**
 * @brief Initialize motor control system
 * @param config Motor configuration details
 * @return ESP_OK on success, error code on failure
 */
esp_err_t motor_control_init(const motor_config_t *config);

/**
 * @brief Set motor speeds
 * @param outputs Motor output values
 * @return ESP_OK on success, error code on failure
 */
esp_err_t motor_control_set_speeds(const motor_outputs_t *outputs);

/**
 * @brief Arm the ESCs (Electronic Speed Controllers)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t motor_control_arm(void);

/**
 * @brief Disarm the ESCs
 * @return ESP_OK on success, error code on failure
 */
esp_err_t motor_control_disarm(void);

/**
 * @brief Calibrate ESCs
 * @param mode Calibration mode (low, high, or standard)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t motor_control_calibrate_escs(esc_calibration_mode_t mode);

/**
 * @brief Get current motor status
 * @param status Pointer to store motor status
 * @return ESP_OK on success, error code on failure
 */
esp_err_t motor_control_get_status(motor_status_t *status);

/**
 * @brief Emergency stop - immediately cut all motor outputs
 * @return ESP_OK on success, error code on failure
 */
esp_err_t motor_control_emergency_stop(void);

/**
 * @brief Set the PWM frequency for motor control
 * @param frequency_hz PWM frequency in Hertz
 * @return ESP_OK on success, error code on failure
 */
esp_err_t motor_control_set_pwm_frequency(uint16_t frequency_hz);

/**
 * @brief Get the last error encountered by the motor control driver
 * @return Last error code
 */
motor_error_t motor_control_get_last_error(void);

/**
 * @brief Apply specific motor mixing for different frame configurations
 * @param frame_type Quadcopter frame configuration
 * @param inputs Raw control inputs
 * @param outputs Processed motor outputs
 * @return ESP_OK on success, error code on failure
 */
esp_err_t motor_control_apply_frame_mixing(
    motor_frame_config_t frame_type,
    const float *inputs,
    motor_outputs_t *outputs
);

#endif // MOTOR_CONTROL_DRIVER_H