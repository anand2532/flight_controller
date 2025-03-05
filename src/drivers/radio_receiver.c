// src/drivers/radio_receiver.c
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <driver/adc.h>
#include <esp_log.h>

#include "drivers/radio_receiver.h"
#include "config.h"

#define TAG "RADIO_RECEIVER"

// Internal radio receiver context
typedef struct {
    radio_receiver_config_t configuration;
    receiver_input_t last_input;
    esp_err_t last_error;
    bool is_calibrated;
    uint32_t signal_timeout_ms;
    uint16_t failsafe_values[MAX_RECEIVER_CHANNELS];
} radio_receiver_context_t;

// Static instance of radio receiver context
static radio_receiver_context_t receiver_ctx = {
    .signal_timeout_ms = 500,  // 500ms signal loss triggers failsafe
    .is_calibrated = false
};

// Internal helper functions
static esp_err_t decode_ppm_signal(receiver_input_t *input);
static esp_err_t normalize_channel_values(receiver_input_t *input);
static flight_mode_t determine_flight_mode(const receiver_input_t *input);

esp_err_t radio_receiver_init(const radio_receiver_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Validate PPM input pin
    if (!GPIO_IS_VALID_GPIO(config->ppm_input_pin)) {
        ESP_LOGE(TAG, "Invalid PPM input GPIO pin");
        return ESP_ERR_INVALID_ARG;
    }

    // Configure PPM input pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << config->ppm_input_pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PPM input pin");
        return ret;
    }

    // Configure RSSI pin if specified
    if (config->rssi_input_pin != GPIO_NUM_NC) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    }

    // Store configuration
    memcpy(&receiver_ctx.configuration, config, sizeof(radio_receiver_config_t));

    // Set default failsafe values
    for (int i = 0; i < MAX_RECEIVER_CHANNELS; i++) {
        receiver_ctx.failsafe_values[i] = PPM_NEUTRAL_PULSE_WIDTH;
    }

    ESP_LOGI(TAG, "Radio receiver initialized");
    return ESP_OK;
}

esp_err_t radio_receiver_read(receiver_input_t *input) {
    if (input == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Decode PPM signal
    esp_err_t ret = decode_ppm_signal(input);
    if (ret != ESP_OK) {
        // Signal lost, use failsafe
        memset(input->channels, 0, sizeof(input->channels));
        for (int i = 0; i < MAX_RECEIVER_CHANNELS; i++) {
            input->channels[i] = receiver_ctx.failsafe_values[i];
        }
        input->signal_lost = true;
        return ret;
    }

    // Normalize channel values
    ret = normalize_channel_values(input);
    if (ret != ESP_OK) {
        return ret;
    }

    // Determine flight mode
    input->current_flight_mode = determine_flight_mode(input);

    // Update last input
    memcpy(&receiver_ctx.last_input, input, sizeof(receiver_input_t));

    return ESP_OK;
}

esp_err_t radio_receiver_calibrate(radio_receiver_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Starting receiver calibration");
    
    // Collect calibration data
    receiver_input_t calibration_input;
    for (int i = 0; i < 500; i++) {  // Collect 500 samples
        esp_err_t ret = radio_receiver_read(&calibration_input);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Calibration failed: signal lost");
            return ret;
        }

        // Update min/max for each channel
        for (int ch = 0; ch < MAX_RECEIVER_CHANNELS; ch++) {
            if (i == 0) {
                config->channel_min[ch] = calibration_input.channels[ch];
                config->channel_max[ch] = calibration_input.channels[ch];
            } else {
                config->channel_min[ch] = 
                    fminf(config->channel_min[ch], calibration_input.channels[ch]);
                config->channel_max[ch] = 
                    fmaxf(config->channel_max[ch], calibration_input.channels[ch]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between samples
    }

    // Update configuration
    memcpy(&receiver_ctx.configuration, config, sizeof(radio_receiver_config_t));
    receiver_ctx.is_calibrated = true;

    ESP_LOGI(TAG, "Receiver calibration complete");
    return ESP_OK;
}

bool radio_receiver_is_signal_valid(void) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (current_time - receiver_ctx.last_input.last_valid_signal_time) 
           < receiver_ctx.signal_timeout_ms;
}

esp_err_t radio_receiver_get_rssi(float *rssi) {
    if (rssi == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read RSSI from ADC if configured
    if (receiver_ctx.configuration.rssi_input_pin != GPIO_NUM_NC) {
        int raw_rssi = adc1_get_raw(ADC1_CHANNEL_0);
        *rssi = (raw_rssi / 4096.0f) * 100.0f;  // Convert to percentage
        return ESP_OK;
    }

    ESP_LOGW(TAG, "RSSI not configured");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t radio_receiver_set_failsafe(const uint16_t *failsafe_values) {
    if (failsafe_values == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(receiver_ctx.failsafe_values, failsafe_values, 
           sizeof(receiver_ctx.failsafe_values));

    return ESP_OK;
}

// Internal helper function to decode PPM signal
static esp_err_t decode_ppm_signal(receiver_input_t *input) {
    // Placeholder for actual PPM decoding
    // In a real implementation, this would use interrupts or RMT 
    // to decode the PPM signal from the receiver
    
    // Simulated implementation for demonstration
    static uint16_t simulated_channels[MAX_RECEIVER_CHANNELS] = {
        1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000
    };

    memcpy(input->channels, simulated_channels, sizeof(simulated_channels));
    input->signal_lost = false;
    input->last_valid_signal_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    return ESP_OK;
}

// Normalize channel values to -1.0 to 1.0 range
static esp_err_t normalize_channel_values(receiver_input_t *input) {
    for (int i = 0; i < MAX_RECEIVER_CHANNELS; i++) {
        // Apply channel inversion if configured
        uint16_t channel_value = input->channels[i];
        if (receiver_ctx.configuration.invert_channels[i]) {
            channel_value = PPM_MIN_PULSE_WIDTH + 
                            (PPM_MAX_PULSE_WIDTH - channel_value);
        }

        // Normalize to -1.0 to 1.0 range
        float normalized = 2.0f * (channel_value - PPM_MIN_PULSE_WIDTH) / 
                           (PPM_MAX_PULSE_WIDTH - PPM_MIN_PULSE_WIDTH) - 1.0f;
        
        input->normalized_channels[i] = normalized;
    }

    return ESP_OK;
}

// Determine flight mode based on aux channel
static flight_mode_t determine_flight_mode(const receiver_input_t *input) {
    // Example flight mode selection logic
    float aux1_value = input->normalized_channels[AUX1_CHANNEL];

    if (aux1_value < -0.5f) {
        return FLIGHT_MODE_MANUAL;
    } else if (aux1_value < 0.0f) {
        return FLIGHT_MODE_STABILIZE;
    } else if (aux1_value < 0.5f) {
        return FLIGHT_MODE_ALTITUDE_HOLD;
    } else {
        return FLIGHT_MODE_RETURN_TO_HOME;
    }
}

esp_err_t radio_receiver_get_last_error(void) {
    return receiver_ctx.last_error;
}