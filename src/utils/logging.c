// src/utils/logging.c
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <driver/uart.h>

#include "utils/logging.h"
#include "config.h"

#define TAG "QUADCOPTER_LOGGING"

// Logging system context
typedef struct {
    logging_config_t config;
    log_entry_t log_buffer[MAX_LOG_BUFFER_SIZE];
    size_t log_buffer_index;
    esp_err_t last_error;
    log_level_t min_log_level;
    bool is_initialized;
    SemaphoreHandle_t log_mutex;
} logging_context_t;

// Static logging context
static logging_context_t log_ctx = {0};

// Convert log level to string
static const char* log_level_to_string(log_level_t level) {
    switch (level) {
        case LOG_LEVEL_DEBUG:    return "DEBUG";
        case LOG_LEVEL_INFO:     return "INFO";
        case LOG_LEVEL_WARNING:  return "WARNING";
        case LOG_LEVEL_ERROR:    return "ERROR";
        case LOG_LEVEL_CRITICAL: return "CRITICAL";
        default:                 return "UNKNOWN";
    }
}

// Convert log type to string
static const char* log_type_to_string(log_type_t type) {
    switch (type) {
        case LOG_TYPE_SYSTEM:          return "SYSTEM";
        case LOG_TYPE_SENSOR:          return "SENSOR";
        case LOG_TYPE_NAVIGATION:      return "NAVIGATION";
        case LOG_TYPE_FLIGHT_CONTROL:  return "FLIGHT_CONTROL";
        case LOG_TYPE_BATTERY:         return "BATTERY";
        case LOG_TYPE_COMMUNICATION:   return "COMMUNICATION";
        case LOG_TYPE_CUSTOM:          return "CUSTOM";
        default:                       return "UNKNOWN";
    }
}

esp_err_t logging_init(const logging_config_t *config) {
    // Create mutex for thread-safe logging
    log_ctx.log_mutex = xSemaphoreCreateMutex();
    if (log_ctx.log_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Use default configuration if not provided
    if (config) {
        memcpy(&log_ctx.config, config, sizeof(logging_config_t));
    } else {
        // Set default configuration
        log_ctx.config.console_output_enabled = true;
        log_ctx.config.sd_card_logging_enabled = false;
        log_ctx.config.telemetry_logging_enabled = false;
        log_ctx.config.min_log_level = LOG_LEVEL_INFO;
        snprintf(log_ctx.config.log_file_path, 
                 sizeof(log_ctx.config.log_file_path), 
                 "/sdcard/quadcopter_log.txt");
    }

    // Set minimum log level
    log_ctx.min_log_level = log_ctx.config.min_log_level;

    // Clear log buffer
    memset(log_ctx.log_buffer, 0, sizeof(log_ctx.log_buffer));
    log_ctx.log_buffer_index = 0;

    log_ctx.is_initialized = true;

    ESP_LOGI(TAG, "Logging system initialized");
    return ESP_OK;
}

esp_err_t log_message(
    log_level_t level, 
    log_type_t type, 
    const char *format, 
    ...
) {
    // Check if logging is initialized and level is sufficient
    if (!log_ctx.is_initialized || level < log_ctx.min_log_level) {
        return ESP_OK;
    }

    // Take mutex
    if (xSemaphoreTake(log_ctx.log_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Prepare log entry
    log_entry_t entry = {0};
    
    // Set timestamp
    entry.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    entry.level = level;
    entry.type = type;

    // Format log message
    va_list args;
    va_start(args, format);
    vsnprintf(entry.message, sizeof(entry.message), format, args);
    va_end(args);

    // Store in circular buffer
    log_ctx.log_buffer[log_ctx.log_buffer_index] = entry;
    log_ctx.log_buffer_index = 
        (log_ctx.log_buffer_index + 1) % MAX_LOG_BUFFER_SIZE;

    // Console output if enabled
    if (log_ctx.config.console_output_enabled) {
        esp_log_level_t esp_level;
        switch (level) {
            case LOG_LEVEL_DEBUG:    esp_level = ESP_LOG_DEBUG; break;
            case LOG_LEVEL_INFO:     esp_level = ESP_LOG_INFO; break;
            case LOG_LEVEL_WARNING:  esp_level = ESP_LOG_WARN; break;
            case LOG_LEVEL_ERROR:    esp_level = ESP_LOG_ERROR; break;
            case LOG_LEVEL_CRITICAL: esp_level = ESP_LOG_ERROR; break;
            default:                 esp_level = ESP_LOG_INFO; break;
        }

        ESP_LOG_LEVEL(esp_level, 
                      log_type_to_string(type), 
                      "%s: %s", 
                      log_level_to_string(level), 
                      entry.message);
    }

    // Release mutex
    xSemaphoreGive(log_ctx.log_mutex);

    return ESP_OK;
}

esp_err_t logging_get_recent_entries(
    log_entry_t *entries, 
    size_t max_entries, 
    size_t *num_retrieved
) {
    if (!entries || !num_retrieved) {
        return ESP_ERR_INVALID_ARG;
    }

    // Take mutex
    if (xSemaphoreTake(log_ctx.log_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Determine number of entries to retrieve
    *num_retrieved = (max_entries < MAX_LOG_BUFFER_SIZE) ? 
                     max_entries : MAX_LOG_BUFFER_SIZE;

    // Copy entries (circular buffer logic)
    size_t start_index = (log_ctx.log_buffer_index >= *num_retrieved) ?
                         (log_ctx.log_buffer_index - *num_retrieved) :
                         (MAX_LOG_BUFFER_SIZE - (*num_retrieved - log_ctx.log_buffer_index));

    for (size_t i = 0; i < *num_retrieved; i++) {
        entries[i] = log_ctx.log_buffer[(start_index + i) % MAX_LOG_BUFFER_SIZE];
    }

    // Release mutex
    xSemaphoreGive(log_ctx.log_mutex);

    return ESP_OK;
}

esp_err_t logging_clear_buffer(void) {
    // Take mutex
    if (xSemaphoreTake(log_ctx.log_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Clear log buffer
    memset(log_ctx.log_buffer, 0, sizeof(log_ctx.log_buffer));
    log_ctx.log_buffer_index = 0;

    // Release mutex
    xSemaphoreGive(log_ctx.log_mutex);

    return ESP_OK;
}

esp_err_t logging_set_min_level(log_level_t level) {
    log_ctx.min_log_level = level;
    return ESP_OK;
}

esp_err_t logging_export_logs(const char *filepath) {
    // Placeholder for SD card logging implementation
    // In a real implementation, this would write logs to a file
    log_message(LOG_LEVEL_INFO, LOG_TYPE_SYSTEM, 
                "Attempting to export logs to %s", filepath);
    
    return ESP_OK;
}

esp_err_t logging_get_last_error(void) {
    return log_ctx.last_error;
}