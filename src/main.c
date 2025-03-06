// src/main.c
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <nvs_flash.h>

// Project-specific headers
#include "config.h"
#include "types.h"
#include "utils/logging.h"
#include "utils/calibration.h"
#include "drivers/mpu6050.h"
#include "drivers/motor_control.h"
#include "drivers/radio_receiver.h"
#include "drivers/battery_monitor.h"
#include "control/pid.h"
#include "control/stabilization.h"
#include "control/navigation.h"

// Task priorities
#define TASK_PRIORITY_SENSOR_READ     (configMAX_PRIORITIES - 1)
#define TASK_PRIORITY_FLIGHT_CONTROL  (configMAX_PRIORITIES - 2)
#define TASK_PRIORITY_TELEMETRY       (configMAX_PRIORITIES - 3)
#define TASK_PRIORITY_SAFETY_MONITOR  (configMAX_PRIORITIES - 4)

// Task stack sizes
#define TASK_STACK_SIZE_SENSOR_READ   4096
#define TASK_STACK_SIZE_FLIGHT_CONTROL 8192
#define TASK_STACK_SIZE_TELEMETRY     4096
#define TASK_STACK_SIZE_SAFETY_MONITOR 4096

// Global system context
typedef struct {
    mpu6050_data_t sensor_data;
    receiver_input_t receiver_input;
    battery_measurement_t battery_status;
    
    stabilization_system_t stabilization;
    navigation_system_t navigation;
    
    system_error_t current_error;
    bool is_armed;
} quadcopter_system_t;

// Global system instance
static quadcopter_system_t quad_system = {0};

// Tag for logging
static const char* TAG = "QUADCOPTER_MAIN";

// Function prototypes
void system_init(void);
void sensor_read_task(void *pvParameters);
void flight_control_task(void *pvParameters);
void telemetry_task(void *pvParameters);
void safety_monitor_task(void *pvParameters);

// System Initialization
void system_init(void) {
    esp_err_t ret;

    // Initialize Non-Volatile Storage
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize logging system
    logging_config_t log_config = {
        .console_output_enabled = true,
        .sd_card_logging_enabled = false,
        .min_log_level = LOG_LEVEL_DEBUG
    };
    ESP_ERROR_CHECK(logging_init(&log_config));

    // Perform system calibration
    calibration_init();
    calibration_result_t cal_result;
    ret = calibration_perform_full_system(&cal_result);
    if (ret != ESP_OK) {
        log_message(LOG_LEVEL_ERROR, LOG_TYPE_SYSTEM, 
                    "Full system calibration failed");
    }

    // Initialize key systems
    ESP_ERROR_CHECK(mpu6050_init());
    ESP_ERROR_CHECK(motor_control_init(NULL));
    ESP_ERROR_CHECK(radio_receiver_init(NULL));
    ESP_ERROR_CHECK(battery_monitor_init(NULL, NULL));

    // Initialize stabilization and navigation
    stabilization_config_t stab_config = {
        .mode = STABILIZE_MODE_ANGLE,
        .max_angle_setpoint = 45.0f,
        .max_rate_setpoint = 250.0f
    };
    ESP_ERROR_CHECK(stabilization_init(&quad_system.stabilization, &stab_config));

    navigation_config_t nav_config = {
        .max_horizontal_speed = 10.0f,
        .max_vertical_speed = 5.0f,
        .navigation_tolerance = 2.0f
    };
    ESP_ERROR_CHECK(navigation_init(&quad_system.navigation, &nav_config));

    log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                "Quadcopter system initialized");
}

// Sensor Reading Task
void sensor_read_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(10); // 100 Hz

    while (1) {
        // Read IMU data
        esp_err_t ret = mpu6050_read_scaled_data(&quad_system.sensor_data);
        if (ret != ESP_OK) {
            log_message(LOG_LEVEL_WARNING, LOG_TYPE_SENSOR, 
                        "Failed to read sensor data");
        }

        // Read radio receiver input
        ret = radio_receiver_read(&quad_system.receiver_input);
        if (ret != ESP_OK) {
            log_message(LOG_LEVEL_WARNING, LOG_TYPE_COMMUNICATION, 
                        "Failed to read receiver input");
        }

        // Read battery status
        ret = battery_monitor_read(&quad_system.battery_status);
        if (ret != ESP_OK) {
            log_message(LOG_LEVEL_WARNING, LOG_TYPE_BATTERY, 
                        "Failed to read battery status");
        }

        // Delay to maintain consistent task frequency
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

// Flight Control Task
void flight_control_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(5); // 200 Hz

    while (1) {
        // Check if system is armed
        if (!quad_system.is_armed) {
            vTaskDelay(frequency);
            continue;
        }

        // Check battery status
        if (quad_system.battery_status.status >= BATTERY_STATUS_LOW_VOLTAGE) {
            log_message(LOG_LEVEL_WARNING, LOG_TYPE_BATTERY, 
                        "Low battery, initiating failsafe");
            stabilization_set_mode(&quad_system.stabilization, 
                                   STABILIZE_MODE_ANGLE);
        }

        // Stabilization
        motor_outputs_t motor_outputs;
        esp_err_t ret = stabilize_quadcopter(
            &quad_system.sensor_data, 
            &quad_system.receiver_input, 
            &motor_outputs
        );

        if (ret == ESP_OK) {
            // Apply motor outputs
            motor_control_set_speeds(&motor_outputs);
        } else {
            log_message(LOG_LEVEL_ERROR, LOG_TYPE_FLIGHT_CONTROL, 
                        "Stabilization failed");
        }

        // Delay to maintain consistent task frequency
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

// Telemetry Task
void telemetry_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(100); // 10 Hz

    while (1) {
        // Construct telemetry packet
        telemetry_packet_t telemetry = {
            .timestamp = xTaskGetTickCount(),
            .status = {
                .flight_mode = quad_system.stabilization.config.mode,
                .is_armed = quad_system.is_armed
            },
            .battery = {
                .voltage = quad_system.battery_status.total_voltage,
                .current = quad_system.battery_status.current,
                .remaining_capacity = quad_system.battery_status.state_of_charge
            }
        };

        // Log telemetry (could also be sent via radio)
        log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                    "Telemetry: V=%.2fV, I=%.2fA, SoC=%.2f%%, Mode=%d", 
                    telemetry.battery.voltage,
                    telemetry.battery.current,
                    telemetry.battery.remaining_capacity,
                    telemetry.status.flight_mode);

        // Delay to maintain consistent task frequency
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

// Safety Monitoring Task
void safety_monitor_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50); // 20 Hz

    while (1) {
        // Run safety checks
        struct {
            bool excessive_tilt;
            bool rapid_rotation;
            bool orientation_loss;
        } safety_status;

        esp_err_t ret = stabilization_safety_monitor(
            &quad_system.stabilization, 
            &safety_status
        );

        if (ret != ESP_OK || safety_status.excessive_tilt || 
            safety_status.rapid_rotation || safety_status.orientation_loss) {
            // Trigger emergency procedures
            log_message(LOG_LEVEL_CRITICAL, LOG_TYPE_SYSTEM, 
                        "Safety critical: Emergency shutdown");
            
            // Disarm and stop motors
            motor_control_emergency_stop();
            quad_system.is_armed = false;
        }

        // Delay to maintain consistent task frequency
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

// Main application entry point
void app_main(void) {
    // System initialization
    system_init();

    // Create tasks
    xTaskCreate(sensor_read_task, 
                "Sensor Read", 
                TASK_STACK_SIZE_SENSOR_READ, 
                NULL, 
                TASK_PRIORITY_SENSOR_READ, 
                NULL);

    xTaskCreate(flight_control_task, 
                "Flight Control", 
                TASK_STACK_SIZE_FLIGHT_CONTROL, 
                NULL, 
                TASK_PRIORITY_FLIGHT_CONTROL, 
                NULL);

    xTaskCreate(telemetry_task, 
                "Telemetry", 
                TASK_STACK_SIZE_TELEMETRY, 
                NULL, 
                TASK_PRIORITY_TELEMETRY, 
                NULL);

    xTaskCreate(safety_monitor_task, 
                "Safety Monitor", 
                TASK_STACK_SIZE_SAFETY_MONITOR, 
                NULL, 
                TASK_PRIORITY_SAFETY_MONITOR, 
                NULL);

    // Initial log
    log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                "Quadcopter firmware v%s started", 
                QUADCOPTER_FIRMWARE_VERSION);
}