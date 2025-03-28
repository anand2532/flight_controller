#include <math.h>
#include <string.h>
#include <esp_log.h>

#include "control/stabilization.h"
#include "config.h"
#include "drivers/radio_receiver.h"

#define TAG "QUADCOPTER_STABILIZATION"

// Default stabilization configuration
static const stabilization_config_t DEFAULT_STABILIZATION_CONFIG = {
    .mode = STABILIZE_MODE_ANGLE,
    .max_angle_setpoint = 45.0f,
    .max_rate_setpoint = 250.0f,
    .acro_rate_multiplier = 1.0f,
    .horizon_mode_strength = 0.5f
};

// Internal helper functions
static float constrain_angle(float angle, float limit);
static float compute_rate_setpoint(
    float stick_input, 
    float max_rate, 
    stabilization_mode_t mode
);

esp_err_t stabilization_init(
    stabilization_system_t *system,
    const stabilization_config_t *config
) {
    if (system == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Use provided config or default
    const stabilization_config_t *use_config = 
        config ? config : &DEFAULT_STABILIZATION_CONFIG;

    // Initialize PIDs with default parameters
    pid_init(&system->roll_pid, PID_ROLL, NULL, 0.01f);
    pid_init(&system->pitch_pid, PID_PITCH, NULL, 0.01f);
    pid_init(&system->yaw_pid, PID_YAW, NULL, 0.01f);

    // Copy configuration
    memcpy(&system->config, use_config, sizeof(stabilization_config_t));

    // Initialize as disarmed
    system->is_armed = false;

    ESP_LOGI(TAG, "Stabilization system initialized");
    return ESP_OK;
}

esp_err_t stabilize_quadcopter(
    const mpu6050_data_t *sensor_data,
    const receiver_input_t *receiver_input,
    motor_outputs_t *motor_outputs
) {
    // Validate inputs
    if (!sensor_data || !receiver_input || !motor_outputs) {
        return ESP_ERR_INVALID_ARG;
    }

    // Compute current orientation
    quadcopter_orientation_t orientation;
    esp_err_t ret = compute_quadcopter_orientation(sensor_data, &orientation);
    if (ret != ESP_OK) {
        return ret;
    }

    // Extract receiver inputs
    float throttle = receiver_input->normalized_channels[THROTTLE_CHANNEL];
    float roll_input = receiver_input->normalized_channels[ROLL_CHANNEL];
    float pitch_input = receiver_input->normalized_channels[PITCH_CHANNEL];
    float yaw_input = receiver_input->normalized_channels[YAW_CHANNEL];

    // Compute setpoints based on stabilization mode
    float roll_setpoint, pitch_setpoint, yaw_setpoint;

    switch (receiver_input->current_flight_mode) {
        case FLIGHT_MODE_MANUAL:
            // Direct pass-through of stick inputs
            roll_setpoint = roll_input * DEFAULT_STABILIZATION_CONFIG.max_angle_setpoint;
            pitch_setpoint = pitch_input * DEFAULT_STABILIZATION_CONFIG.max_angle_setpoint;
            yaw_setpoint = yaw_input * DEFAULT_STABILIZATION_CONFIG.max_rate_setpoint;
            break;

        case FLIGHT_MODE_STABILIZE:
            // Angle mode - convert stick input to angle setpoint
            roll_setpoint = roll_input * DEFAULT_STABILIZATION_CONFIG.max_angle_setpoint;
            pitch_setpoint = pitch_input * DEFAULT_STABILIZATION_CONFIG.max_angle_setpoint;
            yaw_setpoint = yaw_input * DEFAULT_STABILIZATION_CONFIG.max_rate_setpoint;
            break;

        default:
            // Default to level flight
            roll_setpoint = 0.0f;
            pitch_setpoint = 0.0f;
            yaw_setpoint = 0.0f;
            break;
    }

    // Compute PID corrections
    float roll_correction = pid_compute(
        &motor_ctx.roll_pid, 
        roll_setpoint, 
        orientation.roll
    );

    float pitch_correction = pid_compute(
        &motor_ctx.pitch_pid, 
        pitch_setpoint, 
        orientation.pitch
    );

    float yaw_correction = pid_compute(
        &motor_ctx.yaw_pid, 
        yaw_setpoint, 
        orientation.yaw_rate
    );

    // Combine inputs for motor mixing
    float mixer_inputs[3] = {
        throttle,      // Throttle
        roll_correction,  // Roll correction
        pitch_correction  // Pitch correction
    };

    // Apply frame mixing to get motor outputs
    ret = motor_control_apply_frame_mixing(
        QUADCOPTER_X_CONFIG,
        mixer_inputs,
        motor_outputs
    );

    return ret;
}

esp_err_t compute_quadcopter_orientation(
    const mpu6050_data_t *sensor_data,
    quadcopter_orientation_t *orientation
) {
    if (!sensor_data || !orientation) {
        return ESP_ERR_INVALID_ARG;
    }

    // Compute roll and pitch using accelerometer data
    orientation->roll = atan2f(
        sensor_data->scaled.accel_y, 
        sensor_data->scaled.accel_z
    ) * 180.0f / M_PI;

    orientation->pitch = atan2f(
        -sensor_data->scaled.accel_x, 
        sqrtf(
            sensor_data->scaled.accel_y * sensor_data->scaled.accel_y + 
            sensor_data->scaled.accel_z * sensor_data->scaled.accel_z
        )
    ) * 180.0f / M_PI;

    // Use gyroscope for angular rates
    orientation->roll_rate = sensor_data->scaled.gyro_x;
    orientation->pitch_rate = sensor_data->scaled.gyro_y;
    orientation->yaw_rate = sensor_data->scaled.gyro_z;

    return ESP_OK;
}

esp_err_t stabilization_set_mode(
    stabilization_system_t *system, 
    stabilization_mode_t mode
) {
    if (!system) {
        return ESP_ERR_INVALID_ARG;
    }

    system->config.mode = mode;
    
    // Reset PIDs when changing mode
    pid_reset(&system->roll_pid);
    pid_reset(&system->pitch_pid);
    pid_reset(&system->yaw_pid);

    ESP_LOGI(TAG, "Stabilization mode changed to %d", mode);
    return ESP_OK;
}

esp_err_t stabilization_set_arm_state(
    stabilization_system_t *system, 
    bool arm
) {
    if (!system) {
        return ESP_ERR_INVALID_ARG;
    }

    system->is_armed = arm;

    // Reset PIDs when arming/disarming
    if (!arm) {
        pid_reset(&system->roll_pid);
        pid_reset(&system->pitch_pid);
        pid_reset(&system->yaw_pid);
    }

    ESP_LOGI(TAG, "Stabilization system %s", arm ? "ARMED" : "DISARMED");
    return ESP_OK;
}

esp_err_t stabilization_reset(
    stabilization_system_t *system
) {
    if (!system) {
        return ESP_ERR_INVALID_ARG;
    }

    // Reset PIDs
    pid_reset(&system->roll_pid);
    pid_reset(&system->pitch_pid);
    pid_reset(&system->yaw_pid);

    // Reset orientation
    memset(&system->current_orientation, 0, sizeof(quadcopter_orientation_t));

    // Reset to default configuration
    memcpy(&system->config, &DEFAULT_STABILIZATION_CONFIG, 
           sizeof(stabilization_config_t));

    // Disarm
    system->is_armed = false;

    ESP_LOGI(TAG, "Stabilization system reset");
    return ESP_OK;
}

esp_err_t stabilization_tune_pids(
    stabilization_system_t *system,
    const pid_params_t *roll_pid,
    const pid_params_t *pitch_pid,
    const pid_params_t *yaw_pid
) {
    if (!system) {
        return ESP_ERR_INVALID_ARG;
    }

    // Tune individual PIDs if provided
    if (roll_pid) {
        pid_tune(&system->roll_pid, roll_pid);
    }
    if (pitch_pid) {
        pid_tune(&system->pitch_pid, pitch_pid);
    }
    if (yaw_pid) {
        pid_tune(&system->yaw_pid, yaw_pid);
    }

    ESP_LOGI(TAG, "PID controllers tuned");
    return ESP_OK;
}

// Helper function to constrain angle
static float constrain_angle(float angle, float limit) {
    return fmaxf(-limit, fminf(limit, angle));
}

// Helper function to compute rate setpoint
static float compute_rate_setpoint(
    float stick_input, 
    float max_rate, 
    stabilization_mode_t mode
) {
    float rate_setpoint;

    switch (mode) {
        case STABILIZE_MODE_MANUAL:
            // Direct rate control
            rate_setpoint = stick_input * max_rate;
            break;

        case STABILIZE_MODE_ANGLE:
            // Angle control
            rate_setpoint = stick_input * max_rate;
            break;

        case STABILIZE_MODE_HORIZON:
            // Hybrid angle/rate mode
            rate_setpoint = stick_input * max_rate;
            break;

        case STABILIZE_MODE_ACRO:
            // Rate mode with rate multiplier
            rate_setpoint = stick_input * max_rate;
            break;
    }

    return rate_setpoint;
}

// Advanced stabilization diagnostics
esp_err_t stabilization_diagnostic_check(
    stabilization_system_t *system,
    struct {
        bool pid_controllers_functional;
        float max_angle_deviation;
        float max_rate_deviation;
        bool orientation_consistent;
    } *diagnostic_report
) {
    if (!system || !diagnostic_report) {
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize diagnostic report
    memset(diagnostic_report, 0, sizeof(*diagnostic_report));

    // Check PID controller functionality
    pid_state_t roll_state, pitch_state, yaw_state;
    pid_get_state(&system->roll_pid, &roll_state);
    pid_get_state(&system->pitch_pid, &pitch_state);
    pid_get_state(&system->yaw_pid, &yaw_state);

    diagnostic_report->pid_controllers_functional = 
        (fabsf(roll_state.output) < system->config.max_angle_setpoint &&
         fabsf(pitch_state.output) < system->config.max_angle_setpoint &&
         fabsf(yaw_state.output) < system->config.max_rate_setpoint);

    // Analyze orientation consistency
    quadcopter_orientation_t *orientation = &system->current_orientation;
    diagnostic_report->max_angle_deviation = fmaxf(
        fabsf(orientation->roll), 
        fabsf(orientation->pitch)
    );

    diagnostic_report->max_rate_deviation = fmaxf(
        fmaxf(
            fabsf(orientation->roll_rate), 
            fabsf(orientation->pitch_rate)
        ),
        fabsf(orientation->yaw_rate)
    );

    // Check orientation consistency
    diagnostic_report->orientation_consistent = 
        (diagnostic_report->max_angle_deviation < system->config.max_angle_setpoint * 2.0f &&
         diagnostic_report->max_rate_deviation < system->config.max_rate_setpoint * 2.0f);

    return ESP_OK;
}

// Advanced filtering and sensor fusion
esp_err_t apply_complementary_filter(
    const mpu6050_data_t *sensor_data,
    quadcopter_orientation_t *filtered_orientation,
    float alpha
) {
    if (!sensor_data || !filtered_orientation) {
        return ESP_ERR_INVALID_ARG;
    }

    // Accelerometer-based angle calculation
    float accel_roll = atan2f(
        sensor_data->scaled.accel_y, 
        sensor_data->scaled.accel_z
    ) * 180.0f / M_PI;

    float accel_pitch = atan2f(
        -sensor_data->scaled.accel_x, 
        sqrtf(
            sensor_data->scaled.accel_y * sensor_data->scaled.accel_y + 
            sensor_data->scaled.accel_z * sensor_data->scaled.accel_z
        )
    ) * 180.0f / M_PI;

    // Gyroscope-based angle calculation (integration)
    static float gyro_roll = 0, gyro_pitch = 0;
    gyro_roll += sensor_data->scaled.gyro_x * 0.01f;  // Assuming 10ms update rate
    gyro_pitch += sensor_data->scaled.gyro_y * 0.01f;

    // Complementary filter
    filtered_orientation->roll = 
        (1.0f - alpha) * (filtered_orientation->roll + 
        sensor_data->scaled.gyro_x * 0.01f) + 
        alpha * accel_roll;

    filtered_orientation->pitch = 
        (1.0f - alpha) * (filtered_orientation->pitch + 
        sensor_data->scaled.gyro_y * 0.01f) + 
        alpha * accel_pitch;

    // Direct gyroscope rates
    filtered_orientation->roll_rate = sensor_data->scaled.gyro_x;
    filtered_orientation->pitch_rate = sensor_data->scaled.gyro_y;
    filtered_orientation->yaw_rate = sensor_data->scaled.gyro_z;

    return ESP_OK;
}

// Safety monitoring and failsafe mechanisms
esp_err_t stabilization_safety_monitor(
    stabilization_system_t *system,
    struct {
        bool excessive_tilt;
        bool rapid_rotation;
        bool orientation_loss;
    } *safety_status
) {
    if (!system || !safety_status) {
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize safety status
    memset(safety_status, 0, sizeof(*safety_status));

    quadcopter_orientation_t *orientation = &system->current_orientation;

    // Check for excessive tilt
    safety_status->excessive_tilt = 
        (fabsf(orientation->roll) > 45.0f || 
         fabsf(orientation->pitch) > 45.0f);

    // Check for rapid rotation
    safety_status->rapid_rotation = 
        (fabsf(orientation->roll_rate) > 300.0f ||
         fabsf(orientation->pitch_rate) > 300.0f ||
         fabsf(orientation->yaw_rate) > 300.0f);

    // Check for potential orientation loss
    safety_status->orientation_loss = 
        (fabsf(orientation->roll) > 60.0f || 
         fabsf(orientation->pitch) > 60.0f);

    // If any critical safety condition is met, trigger failsafe
    if (safety_status->excessive_tilt || 
        safety_status->rapid_rotation || 
        safety_status->orientation_loss) {
        
        // Trigger emergency procedures
        stabilization_reset(system);
        motor_control_emergency_stop();
        
        ESP_LOGE(TAG, "SAFETY CRITICAL: Emergency shutdown activated");
    }

    return ESP_OK;
}

// Logging and telemetry
esp_err_t stabilization_log_telemetry(
    const stabilization_system_t *system,
    struct {
        float roll_setpoint;
        float pitch_setpoint;
        float yaw_setpoint;
        float roll_output;
        float pitch_output;
        float yaw_output;
    } *telemetry
) {
    if (!system || !telemetry) {
        return ESP_ERR_INVALID_ARG;
    }

    pid_state_t roll_state, pitch_state, yaw_state;
    pid_get_state(&system->roll_pid, &roll_state);
    pid_get_state(&system->pitch_pid, &pitch_state);
    pid_get_state(&system->yaw_pid, &yaw_state);

    telemetry->roll_setpoint = roll_state.setpoint;
    telemetry->pitch_setpoint = pitch_state.setpoint;
    telemetry->yaw_setpoint = yaw_state.setpoint;

    telemetry->roll_output = roll_state.output;
    telemetry->pitch_output = pitch_state.output;
    telemetry->yaw_output = yaw_state.output;

    // Optional logging
    ESP_LOGI(TAG, "Stabilization Telemetry: "
             "Roll(SP:%.2f, Out:%.2f) "
             "Pitch(SP:%.2f, Out:%.2f) "
             "Yaw(SP:%.2f, Out:%.2f)",
             telemetry->roll_setpoint, telemetry->roll_output,
             telemetry->pitch_setpoint, telemetry->pitch_output,
             telemetry->yaw_setpoint, telemetry->yaw_output);

    return ESP_OK;
}

