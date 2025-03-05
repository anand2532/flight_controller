// include/control/stabilization.h
#ifndef QUADCOPTER_STABILIZATION_H
#define QUADCOPTER_STABILIZATION_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#include "drivers/mpu6050.h"
#include "drivers/radio_receiver.h"
#include "drivers/motor_control.h"
#include "control/pid.h"

// Stabilization modes
typedef enum {
    STABILIZE_MODE_MANUAL,
    STABILIZE_MODE_ANGLE,
    STABILIZE_MODE_HORIZON,
    STABILIZE_MODE_ACRO
} stabilization_mode_t;

// Quadcopter orientation data
typedef struct {
    float roll;      // Roll angle (degrees)
    float pitch;     // Pitch angle (degrees)
    float yaw;       // Yaw angle (degrees)
    float roll_rate;   // Roll angular velocity (degrees/sec)
    float pitch_rate;  // Pitch angular velocity (degrees/sec)
    float yaw_rate;    // Yaw angular velocity (degrees/sec)
} quadcopter_orientation_t;

// Stabilization configuration
typedef struct {
    stabilization_mode_t mode;
    
    // Angle limits
    float max_angle_setpoint;    // Maximum angle setpoint (degrees)
    float max_rate_setpoint;     // Maximum rate setpoint (degrees/sec)
    
    // Acro mode rate multiplier
    float acro_rate_multiplier;
    
    // Horizon mode mixing
    float horizon_mode_strength;
} stabilization_config_t;

// Stabilization system context
typedef struct {
    pid_controller_t roll_pid;
    pid_controller_t pitch_pid;
    pid_controller_t yaw_pid;
    
    stabilization_config_t config;
    quadcopter_orientation_t current_orientation;
    
    bool is_armed;
} stabilization_system_t;

/**
 * @brief Initialize stabilization system
 * @param system Pointer to stabilization system
 * @param config Initial configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t stabilization_init(
    stabilization_system_t *system,
    const stabilization_config_t *config
);

/**
 * @brief Update stabilization based on sensor and receiver input
 * @param system Stabilization system context
 * @param sensor_data Current sensor measurements
 * @param receiver_input Radio receiver input
 * @param motor_outputs Computed motor outputs
 * @return ESP_OK on success, error code on failure
 */
esp_err_t stabilize_quadcopter(
    const mpu6050_data_t *sensor_data,
    const receiver_input_t *receiver_input,
    motor_outputs_t *motor_outputs
);

/**
 * @brief Set stabilization mode
 * @param system Stabilization system
 * @param mode New stabilization mode
 * @return ESP_OK on success, error code on failure
 */
esp_err_t stabilization_set_mode(
    stabilization_system_t *system, 
    stabilization_mode_t mode
);

/**
 * @brief Arm or disarm the stabilization system
 * @param system Stabilization system
 * @param arm Flag to arm or disarm
 * @return ESP_OK on success, error code on failure
 */
esp_err_t stabilization_set_arm_state(
    stabilization_system_t *system, 
    bool arm
);

/**
 * @brief Compute quadcopter orientation from sensor data
 * @param sensor_data Sensor measurements
 * @param orientation Computed orientation
 * @return ESP_OK on success, error code on failure
 */
esp_err_t compute_quadcopter_orientation(
    const mpu6050_data_t *sensor_data,
    quadcopter_orientation_t *orientation
);

/**
 * @brief Reset stabilization system to initial state
 * @param system Stabilization system
 * @return ESP_OK on success, error code on failure
 */
esp_err_t stabilization_reset(
    stabilization_system_t *system
);

/**
 * @brief Tune PID controllers for stabilization
 * @param system Stabilization system
 * @param roll_pid Roll PID parameters
 * @param pitch_pid Pitch PID parameters
 * @param yaw_pid Yaw PID parameters
 * @return ESP_OK on success, error code on failure
 */
esp_err_t stabilization_tune_pids(
    stabilization_system_t *system,
    const pid_params_t *roll_pid,
    const pid_params_t *pitch_pid,
    const pid_params_t *yaw_pid
);

#endif // QUADCOPTER_STABILIZATION_H