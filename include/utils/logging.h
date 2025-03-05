// include/utils/logging.h
#ifndef QUADCOPTER_LOGGING_H
#define QUADCOPTER_LOGGING_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

// Maximum log entry size
#define MAX_LOG_ENTRY_SIZE 256
#define MAX_LOG_BUFFER_SIZE 100

// Log levels
typedef enum {
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_CRITICAL
} log_level_t;

// Log entry types
typedef enum {
    LOG_TYPE_SYSTEM,
    LOG_TYPE_SENSOR,
    LOG_TYPE_NAVIGATION,
    LOG_TYPE_FLIGHT_CONTROL,
    LOG_TYPE_BATTERY,
    LOG_TYPE_COMMUNICATION,
    LOG_TYPE_CUSTOM
} log_type_t;

// Log entry structure
typedef struct {
    uint32_t timestamp;       // Timestamp in milliseconds
    log_level_t level;        // Log level
    log_type_t type;          // Log type
    char message[MAX_LOG_ENTRY_SIZE]; // Log message
} log_entry_t;

// Logging configuration
typedef struct {
    bool console_output_enabled;
    bool sd_card_logging_enabled;
    bool telemetry_logging_enabled;
    log_level_t min_log_level;
    char log_file_path[128];
} logging_config_t;

/**
 * @brief Initialize logging system
 * @param config Logging configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t logging_init(const logging_config_t *config);

/**
 * @brief Log a message
 * @param level Log level
 * @param type Log type
 * @param format Printf-style format string
 * @param ... Variable arguments for format string
 * @return ESP_OK on success, error code on failure
 */
esp_err_t log_message(
    log_level_t level, 
    log_type_t type, 
    const char *format, 
    ...
) __attribute__((format(printf, 3, 4)));

/**
 * @brief Retrieve recent log entries
 * @param entries Pointer to store log entries
 * @param max_entries Maximum number of entries to retrieve
 * @param num_retrieved Pointer to store number of entries retrieved
 * @return ESP_OK on success, error code on failure
 */
esp_err_t logging_get_recent_entries(
    log_entry_t *entries, 
    size_t max_entries, 
    size_t *num_retrieved
);

/**
 * @brief Clear log buffer
 * @return ESP_OK on success, error code on failure
 */
esp_err_t logging_clear_buffer(void);

/**
 * @brief Set minimum log level for filtering
 * @param level Minimum log level to record
 * @return ESP_OK on success, error code on failure
 */
esp_err_t logging_set_min_level(log_level_t level);

/**
 * @brief Export logs to a file
 * @param filepath Path to export log file
 * @return ESP_OK on success, error code on failure
 */
esp_err_t logging_export_logs(const char *filepath);

/**
 * @brief Get the last error encountered by the logging system
 * @return Last error code
 */
esp_err_t logging_get_last_error(void);

#endif // QUADCOPTER_LOGGING_H