#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "data_model.h"
#include "esp_err.h"

typedef void (*ble_manager_settings_cb_t)(const wearable_ble_settings_t *settings, void *user_ctx);
typedef void (*ble_manager_command_cb_t)(const wearable_ble_command_t *command, void *user_ctx);

typedef struct {
    const char *device_name;
    uint16_t notify_interval_ms;
    uint8_t buzzer_enabled;
    int16_t body_temp_offset_c_x100;
    uint8_t debug_log_level;
    ble_manager_settings_cb_t settings_cb;
    ble_manager_command_cb_t command_cb;
    void *user_ctx;
} ble_manager_config_t;

esp_err_t ble_manager_init(const ble_manager_config_t *config);
esp_err_t ble_manager_publish_telemetry(const wearable_telemetry_t *telemetry);
bool ble_manager_is_connected(void);
wearable_ble_settings_t ble_manager_get_settings(void);

#endif
