// include/control/pid.h
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

// PID Controller Axis
typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW
} pid_axis_t;

// PID Configuration Structure
typedef struct {
    float kp;        // Proportional gain
    float ki;        // Integral gain
    float kd;        // Derivative gain
    float k_ff;      // Feed-forward gain
    float max_output;// Maximum output limit
    float min_output;// Minimum output limit
    float max_integral; // Maximum integral windup limit
    float min_integral; // Minimum integral windup limit
} pid_params_t;

// PID Controller State
typedef struct {
    float setpoint;      // Desired value
    float measured_value;// Current measured value
    float last_error;    // Previous error for derivative calculation
    float integral;      // Accumulated error
    float derivative;    // Rate of change of error
    float output;        // Computed controller output
} pid_state_t;

// PID Controller Context
typedef struct {
    pid_params_t params;     // PID configuration parameters
    pid_state_t state;       // Current PID state
    pid_axis_t axis;         // Axis this PID controller manages
    float dt;                // Time between updates
    bool is_enabled;         // Controller enabled flag
} pid_controller_t;

/**
 * @brief Initialize a PID controller
 * @param controller Pointer to PID controller structure
 * @param axis Axis this controller will manage
 * @param params Initial PID parameters
 * @param dt Time step between updates
 * @return true if initialization successful, false otherwise
 */
bool pid_init(
    pid_controller_t *controller, 
    pid_axis_t axis, 
    const pid_params_t *params, 
    float dt
);

/**
 * @brief Compute PID controller output
 * @param controller Pointer to PID controller
 * @param setpoint Desired value
 * @param measured_value Current measured value
 * @return Computed controller output
 */
float pid_compute(
    pid_controller_t *controller, 
    float setpoint, 
    float measured_value
);

/**
 * @brief Reset PID controller state
 * @param controller Pointer to PID controller
 */
void pid_reset(pid_controller_t *controller);

/**
 * @brief Enable or disable PID controller
 * @param controller Pointer to PID controller
 * @param enable Flag to enable/disable
 */
void pid_set_enabled(pid_controller_t *controller, bool enable);

/**
 * @brief Update PID gains
 * @param controller Pointer to PID controller
 * @param params New PID parameters
 */
void pid_tune(
    pid_controller_t *controller, 
    const pid_params_t *params
);

/**
 * @brief Get current PID controller state
 * @param controller Pointer to PID controller
 * @param state Pointer to store current state
 */
void pid_get_state(
    const pid_controller_t *controller, 
    pid_state_t *state
);

/**
 * @brief Configure anti-windup method
 * @param controller Pointer to PID controller
 * @param max_integral Maximum integral term
 * @param min_integral Minimum integral term
 */
void pid_set_integral_limits(
    pid_controller_t *controller, 
    float max_integral, 
    float min_integral
);

#endif // PID_CONTROLLER_H