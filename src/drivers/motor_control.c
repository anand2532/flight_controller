// src/drivers/motor_control.c
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/mcpwm.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "drivers/motor_control.h"
#include "config.h"

#define TAG "MOTOR_CONTROL"

// Internal motor control state
typedef struct {
    motor_config_t configuration;
    motor_status_t status;
    motor_error_t last_error;
    mcpwm_unit_t pwm_unit;
    mcpwm_timer_t pwm_timer;
} motor_control_context_t;

// Static instance of motor control context
static motor_control_context_t motor_ctx = {0};

// Internal helper functions
static esp_err_t validate_motor_speeds(const motor_outputs_t *outputs);
static esp_err_t apply_pwm_outputs(const motor_outputs_t *outputs);

// Motor mixing matrices for different frame configurations
static const float X_FRAME_MIXING[MAX_MOTORS][3] = {
    { 1, -1, -1},  // Motor 0
    { 1, -1,  1},  // Motor 1
    { 1,  1, -1},  // Motor 2
    { 1,  1,  1}   // Motor 3
};

static const float PLUS_FRAME_MIXING[MAX_MOTORS][3] = {
    { 1,  0, -1},  // Motor 0 (Front)
    { 1, -1,  0},  // Motor 1 (Right)
    { 1,  0,  1},  // Motor 2 (Back)
    { 1,  1,  0}   // Motor 3 (Left)
};

esp_err_t motor_control_init(const motor_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Validate motor pins
    for (int i = 0; i < MAX_MOTORS; i++) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(config->motor_pins[i])) {
            ESP_LOGE(TAG, "Invalid GPIO pin for motor %d", i);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // Initialize MCPWM unit
    mcpwm_config_t pwm_config = {
        .frequency = config->pwm_frequency,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER
    };

    // Configure PWM for each motor pin
    for (int i = 0; i < MAX_MOTORS; i++) {
        gpio_set_direction(config->motor_pins[i], GPIO_MODE_OUTPUT);
        
        esp_err_t ret = mcpwm_gpio_init(
            motor_ctx.pwm_unit, 
            MCPWM0A + i,  // Use MCPWM channels
            config->motor_pins[i]
        );
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize PWM for motor %d", i);
            return ret;
        }
    }

    // Configure MCPWM
    esp_err_t ret = mcpwm_init(
        motor_ctx.pwm_unit, 
        motor_ctx.pwm_timer, 
        &pwm_config
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MCPWM initialization failed");
        return ret;
    }

    // Store configuration
    memcpy(&motor_ctx.configuration, config, sizeof(motor_config_t));
    motor_ctx.status.is_armed = false;

    ESP_LOGI(TAG, "Motor control initialized successfully");
    return ESP_OK;
}

esp_err_t motor_control_set_speeds(const motor_outputs_t *outputs) {
    // Validate motor outputs
    esp_err_t ret = validate_motor_speeds(outputs);
    if (ret != ESP_OK) {
        return ret;
    }

    // Check if motors are armed
    if (!motor_ctx.status.is_armed) {
        motor_ctx.last_error = MOTOR_ERROR_NOT_ARMED;
        return ESP_ERR_INVALID_STATE;
    }

    // Apply PWM outputs
    ret = apply_pwm_outputs(outputs);
    if (ret != ESP_OK) {
        return ret;
    }

    // Update motor status
    memcpy(motor_ctx.status.current_speeds, outputs->speeds, sizeof(outputs->speeds));
    
    return ESP_OK;
}

esp_err_t motor_control_arm(void) {
    // Set all motors to minimum pulse width
    motor_outputs_t arm_outputs = {0};
    for (int i = 0; i < MAX_MOTORS; i++) {
        arm_outputs.speeds[i] = MOTOR_MIN_PULSE_WIDTH;
    }

    esp_err_t ret = apply_pwm_outputs(&arm_outputs);
    if (ret != ESP_OK) {
        return ret;
    }

    // Wait for ESCs to recognize arming sequence
    vTaskDelay(pdMS_TO_TICKS(1000));

    motor_ctx.status.is_armed = true;
    ESP_LOGI(TAG, "Motors armed");
    return ESP_OK;
}

esp_err_t motor_control_disarm(void) {
    // Set all motors to minimum pulse width
    motor_outputs_t disarm_outputs = {0};
    for (int i = 0; i < MAX_MOTORS; i++) {
        disarm_outputs.speeds[i] = MOTOR_MIN_PULSE_WIDTH;
    }

    esp_err_t ret = apply_pwm_outputs(&disarm_outputs);
    if (ret != ESP_OK) {
        return ret;
    }

    motor_ctx.status.is_armed = false;
    ESP_LOGI(TAG, "Motors disarmed");
    return ESP_OK;
}

// Helper function to validate motor speeds
static esp_err_t validate_motor_speeds(const motor_outputs_t *outputs) {
    for (int i = 0; i < MAX_MOTORS; i++) {
        if (outputs->speeds[i] < MOTOR_MIN_PULSE_WIDTH || 
            outputs->speeds[i] > MOTOR_MAX_PULSE_WIDTH) {
            motor_ctx.last_error = MOTOR_ERROR_INVALID_SPEED;
            ESP_LOGE(TAG, "Invalid speed for motor %d: %d", 
                     i, outputs->speeds[i]);
            return ESP_ERR_INVALID_ARG;
        }
    }
    return ESP_OK;
}

// Helper function to apply PWM outputs
static esp_err_t apply_pwm_outputs(const motor_outputs_t *outputs) {
    for (int i = 0; i < MAX_MOTORS; i++) {
        // Convert pulse width to duty cycle
        float duty_cycle = (outputs->speeds[i] - MOTOR_MIN_PULSE_WIDTH) / 
                           (float)(MOTOR_MAX_PULSE_WIDTH - MOTOR_MIN_PULSE_WIDTH) * 100.0f;
        
        esp_err_t ret = mcpwm_set_duty(
            motor_ctx.pwm_unit, 
            motor_ctx.pwm_timer, 
            MCPWM_OPR_A, 
            duty_cycle
        );
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set PWM for motor %d", i);
            return ret;
        }
    }
    return ESP_OK;
}

// Frame mixing implementation
esp_err_t motor_control_apply_frame_mixing(
    motor_frame_config_t frame_type,
    const float *inputs,
    motor_outputs_t *outputs
) {
    const float (*mixing_matrix)[3];
    
    // Select appropriate mixing matrix
    switch (frame_type) {
        case QUADCOPTER_X_CONFIG:
            mixing_matrix = X_FRAME_MIXING;
            break;
        case QUADCOPTER_PLUS_CONFIG:
            mixing_matrix = PLUS_FRAME_MIXING;
            break;
        default:
            ESP_LOGE(TAG, "Unsupported frame configuration");
            return ESP_ERR_INVALID_ARG;
    }

    // Apply mixing
    for (int i = 0; i < MAX_MOTORS; i++) {
        float motor_value = 
            inputs[0] * mixing_matrix[i][0] +  // Throttle
            inputs[1] * mixing_matrix[i][1] +  // Roll
            inputs[2] * mixing_matrix[i][2];   // Pitch

        // Constrain and map to pulse width
        motor_value = fmaxf(0.0f, fminf(1.0f, motor_value));
        outputs->speeds[i] = (uint16_t)(
            MOTOR_MIN_PULSE_WIDTH + 
            motor_value * (MOTOR_MAX_PULSE_WIDTH - MOTOR_MIN_PULSE_WIDTH)
        );
    }

    return ESP_OK;
}

// Other implementation functions follow a similar pattern