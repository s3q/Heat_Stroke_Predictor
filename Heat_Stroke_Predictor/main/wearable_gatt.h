#ifndef WEARABLE_GATT_H
#define WEARABLE_GATT_H

#include <stdbool.h>
#include <stdint.h>

#include "data_model.h"
#include "esp_err.h"

#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

#define WEARABLE_BLE_SERVICE_UUID_STR "12345678-1234-5678-1234-56789abc0000"
#define WEARABLE_BLE_DEVICE_STATUS_CHAR_UUID_STR "12345678-1234-5678-1234-56789abc0001"
#define WEARABLE_BLE_LIVE_DATA_CHAR_UUID_STR "12345678-1234-5678-1234-56789abc0002"
#define WEARABLE_BLE_RISK_CHAR_UUID_STR "12345678-1234-5678-1234-56789abc0003"
#define WEARABLE_BLE_SETTINGS_CHAR_UUID_STR "12345678-1234-5678-1234-56789abc0004"
#define WEARABLE_BLE_COMMAND_CHAR_UUID_STR "12345678-1234-5678-1234-56789abc0005"

typedef void (*wearable_gatt_settings_cb_t)(const wearable_ble_settings_t *settings, void *user_ctx);
typedef void (*wearable_gatt_command_cb_t)(const wearable_ble_command_t *command, void *user_ctx);

typedef struct {
    wearable_gatt_settings_cb_t settings_cb;
    wearable_gatt_command_cb_t command_cb;
    void *user_ctx;
} wearable_gatt_callbacks_t;

esp_err_t wearable_gatt_init(const wearable_gatt_callbacks_t *callbacks,
                             const wearable_ble_settings_t *initial_settings);

const struct ble_gatt_svc_def *wearable_gatt_service_defs(void);
const ble_uuid128_t *wearable_gatt_service_uuid(void);

void wearable_gatt_set_connection_state(bool connected, uint16_t mtu);
void wearable_gatt_set_advertising_state(bool active);
uint16_t wearable_gatt_get_notify_interval_ms(void);
wearable_ble_settings_t wearable_gatt_get_settings(void);

void wearable_gatt_update_telemetry(const wearable_telemetry_t *telemetry);
esp_err_t wearable_gatt_notify_live(void);
esp_err_t wearable_gatt_notify_risk(void);
esp_err_t wearable_gatt_notify_status(void);

#endif
