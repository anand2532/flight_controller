// include/drivers/radio_receiver.h
#ifndef RADIO_RECEIVER_DRIVER_H
#define RADIO_RECEIVER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

// Maximum number of channels supported
#define MAX_RECEIVER_CHANNELS 8

// PPM configuration
#define PPM_MIN_PULSE_WIDTH      1000    // Minimum pulse width (μs)
#define PPM_MAX_PULSE_WIDTH      2000    // Maximum pulse width (μs)
#define PPM_NEUTRAL_PULSE_WIDTH  1500    // Neutral pulse width (μs)

// Flight mode definitions
typedef enum {
    FLIGHT_MODE_MANUAL,
    FLIGHT_MODE_STABILIZE,
    FLIGHT_MODE_ALTITUDE_HOLD,
    FLIGHT_MODE_RETURN_TO_HOME,
    FLIGHT_MODE_FAILSAFE
} flight_mode_t;

// Receiver channel mapping
typedef enum {
    THROTTLE_CHANNEL = 0,
    ROLL_CHANNEL,
    PITCH_CHANNEL,
    YAW_CHANNEL,
    AUX1_CHANNEL,
    AUX2_CHANNEL,
    AUX3_CHANNEL,
    AUX4_CHANNEL
} receiver_channel_t;

// Radio receiver input structure
typedef struct {
    uint16_t channels[MAX_RECEIVER_CHANNELS];  // Raw channel values
    float normalized_channels[MAX_RECEIVER_CHANNELS];  // Normalized to -1.0 to 1.0
    flight_mode_t current_flight_mode;
    bool signal_lost;
    uint32_t last_valid_signal_time;
} receiver_input_t;

// Radio receiver configuration
typedef struct {
    uint8_t ppm_input_pin;          // GPIO pin for PPM input
    uint8_t rssi_input_pin;         // RSSI (Received Signal Strength Indicator) pin
    bool invert_channels[MAX_RECEIVER_CHANNELS];  // Channel inversion flags
    uint16_t channel_min[MAX_RECEIVER_CHANNELS];  // Minimum pulse width per channel
    uint16_t channel_max[MAX_RECEIVER_CHANNELS];  // Maximum pulse width per channel
} radio_receiver_config_t;

/**
 * @brief Initialize the radio receiver
 * @param config Receiver configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t radio_receiver_init(const radio_receiver_config_t *config);

/**
 * @brief Read current receiver input
 * @param input Pointer to store receiver input data
 * @return ESP_OK on success, error code on failure
 */
esp_err_t radio_receiver_read(receiver_input_t *input);

/**
 * @brief Calibrate receiver channel ranges
 * @param config Pointer to receiver configuration to update
 * @return ESP_OK on success, error code on failure
 */
esp_err_t radio_receiver_calibrate(radio_receiver_config_t *config);

/**
 * @brief Check if receiver signal is valid
 * @return true if signal is valid, false otherwise
 */
bool radio_receiver_is_signal_valid(void);

/**
 * @brief Get current RSSI (Received Signal Strength Indicator)
 * @param rssi Pointer to store RSSI value
 * @return ESP_OK on success, error code on failure
 */
esp_err_t radio_receiver_get_rssi(float *rssi);

/**
 * @brief Set failsafe values for receiver channels
 * @param failsafe_values Array of failsafe pulse widths
 * @return ESP_OK on success, error code on failure
 */
esp_err_t radio_receiver_set_failsafe(const uint16_t *failsafe_values);

/**
 * @brief Get the last error encountered by the radio receiver driver
 * @return Last error code
 */
esp_err_t radio_receiver_get_last_error(void);

#endif // RADIO_RECEIVER_DRIVER_H