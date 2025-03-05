// src/config.c
#include <esp_system.h>
#include <nvs_flash.h>
#include "config.h"

// Namespace for NVS storage
#define CONFIG_NAMESPACE "quadcopter_cfg"

// Default Configurations
const airframe_config_t DEFAULT_AIRFRAME_CONFIG = {
    .frame_type = FRAME_TYPE_X,
    .max_horizontal_speed = 10.0f,
    .max_vertical_speed = 5.0f,
    .max_tilt_angle = 45.0f
};

const pid_gains_config_t DEFAULT_PID_GAINS = {
    .kp_roll = 1.0f,
    .ki_roll = 0.01f,
    .kd_roll = 0.1f,
    .kp_pitch = 1.0f,
    .ki_pitch = 0.01f,
    .kd_pitch = 0.1f,
    .kp_yaw = 1.0f,
    .ki_yaw = 0.01f,
    .kd_yaw = 0.1f
};

void config_init(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Erase and retry if needed
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

esp_err_t config_load(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    // Open NVS namespace
    ret = nvs_open(CONFIG_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    // Load airframe config
    size_t required_size;
    ret = nvs_get_blob(nvs_handle, "airframe_cfg", NULL, &required_size);
    if (ret == ESP_OK) {
        airframe_config_t loaded_airframe_config;
        ret = nvs_get_blob(nvs_handle, "airframe_cfg", &loaded_airframe_config, &required_size);
    }

    // Load PID gains config
    ret = nvs_get_blob(nvs_handle, "pid_gains_cfg", NULL, &required_size);
    if (ret == ESP_OK) {
        pid_gains_config_t loaded_pid_gains;
        ret = nvs_get_blob(nvs_handle, "pid_gains_cfg", &loaded_pid_gains, &required_size);
    }

    nvs_close(nvs_handle);
    return ret;
}

esp_err_t config_save(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret;

    // Open NVS namespace
    ret = nvs_open(CONFIG_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    // Save airframe config
    ret = nvs_set_blob(nvs_handle, "airframe_cfg", &DEFAULT_AIRFRAME_CONFIG, sizeof(airframe_config_t));
    
    // Save PID gains config
    ret = nvs_set_blob(nvs_handle, "pid_gains_cfg", &DEFAULT_PID_GAINS, sizeof(pid_gains_config_t));

    // Commit changes
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);
    return ret;
}