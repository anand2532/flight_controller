// src/control/pid.c
#include <string.h>
#include <math.h>
#include <float.h>
#include <esp_log.h>

#include "control/pid.h"
#include "config.h"

#define TAG "PID_CONTROLLER"

// Default PID parameters for different axes
static const pid_params_t DEFAULT_ROLL_PARAMS = {
    .kp = 1.0f,
    .ki = 0.01f,
    .kd = 0.1f,
    .k_ff = 0.0f,
    .max_output = 45.0f,
    .min_output = -45.0f,
    .max_integral = 10.0f,
    .min_integral = -10.0f
};

static const pid_params_t DEFAULT_PITCH_PARAMS = {
    .kp = 1.0f,
    .ki = 0.01f,
    .kd = 0.1f,
    .k_ff = 0.0f,
    .max_output = 45.0f,
    .min_output = -45.0f,
    .max_integral = 10.0f,
    .min_integral = -10.0f
};

static const pid_params_t DEFAULT_YAW_PARAMS = {
    .kp = 1.0f,
    .ki = 0.01f,
    .kd = 0.1f,
    .k_ff = 0.0f,
    .max_output = 45.0f,
    .min_output = -45.0f,
    .max_integral = 10.0f,
    .min_integral = -10.0f
};

// Helper function to select default parameters
static const pid_params_t* get_default_params(pid_axis_t axis) {
    switch (axis) {
        case PID_ROLL:
            return &DEFAULT_ROLL_PARAMS;
        case PID_PITCH:
            return &DEFAULT_PITCH_PARAMS;
        case PID_YAW:
            return &DEFAULT_YAW_PARAMS;
        default:
            ESP_LOGE(TAG, "Invalid axis selection");
            return &DEFAULT_ROLL_PARAMS;
    }
}

bool pid_init(
    pid_controller_t *controller, 
    pid_axis_t axis, 
    const pid_params_t *params, 
    float dt
) {
    if (controller == NULL) {
        ESP_LOGE(TAG, "NULL controller pointer");
        return false;
    }

    // Use provided params or default if NULL
    const pid_params_t *use_params = (params != NULL) ? params : get_default_params(axis);

    // Initialize controller
    memset(controller, 0, sizeof(pid_controller_t));
    
    // Set axis and parameters
    controller->axis = axis;
    memcpy(&controller->params, use_params, sizeof(pid_params_t));
    
    // Set time step
    controller->dt = (dt > 0.0f) ? dt : 0.01f;  // Default to 10ms if invalid
    
    // Enable controller by default
    controller->is_enabled = true;

    ESP_LOGI(TAG, "PID Controller initialized for axis %d", axis);
    return true;
}

float pid_compute(
    pid_controller_t *controller, 
    float setpoint, 
    float measured_value
) {
    // Check if controller is enabled
    if (!controller->is_enabled) {
        return 0.0f;
    }

    // Compute error
    float error = setpoint - measured_value;

    // Proportional term
    float p_term = controller->params.kp * error;

    // Integral term with anti-windup
    controller->state.integral += error * controller->dt;
    
    // Clamp integral term
    controller->state.integral = fmaxf(
        controller->params.min_integral, 
        fminf(controller->params.max_integral, controller->state.integral)
    );
    
    float i_term = controller->params.ki * controller->state.integral;

    // Derivative term (with low-pass filtering)
    float derivative = (error - controller->state.last_error) / controller->dt;
    controller->state.derivative = derivative;
    float d_term = controller->params.kd * derivative;

    // Feed-forward term
    float ff_term = controller->params.k_ff * setpoint;

    // Compute total output
    float output = p_term + i_term + d_term + ff_term;

    // Clamp output
    output = fmaxf(
        controller->params.min_output, 
        fminf(controller->params.max_output, output)
    );

    // Store error for next iteration
    controller->state.last_error = error;
    controller->state.setpoint = setpoint;
    controller->state.measured_value = measured_value;
    controller->state.output = output;

    return output;
}

void pid_reset(pid_controller_t *controller) {
    if (controller == NULL) return;

    // Reset integral and last error
    controller->state.integral = 0.0f;
    controller->state.last_error = 0.0f;
    controller->state.derivative = 0.0f;
    controller->state.output = 0.0f;
}

void pid_set_enabled(pid_controller_t *controller, bool enable) {
    if (controller == NULL) return;

    // Set enabled state and reset if enabling
    if (enable && !controller->is_enabled) {
        pid_reset(controller);
    }
    
    controller->is_enabled = enable;
}

void pid_tune(
    pid_controller_t *controller, 
    const pid_params_t *params
) {
    if (controller == NULL || params == NULL) return;

    // Update gains
    memcpy(&controller->params, params, sizeof(pid_params_t));
    
    // Reset controller to apply new gains
    pid_reset(controller);

    ESP_LOGI(TAG, "PID Controller tuned: Kp=%.3f, Ki=%.3f, Kd=%.3f", 
             params->kp, params->ki, params->kd);
}

void pid_get_state(
    const pid_controller_t *controller, 
    pid_state_t *state
) {
    if (controller == NULL || state == NULL) return;

    // Copy current state
    memcpy(state, &controller->state, sizeof(pid_state_t));
}

void pid_set_integral_limits(
    pid_controller_t *controller, 
    float max_integral, 
    float min_integral
) {
    if (controller == NULL) return;

    // Update integral limits
    controller->params.max_integral = max_integral;
    controller->params.min_integral = min_integral;

    // Clamp existing integral
    controller->state.integral = fmaxf(
        min_integral, 
        fminf(max_integral, controller->state.integral)
    );
}