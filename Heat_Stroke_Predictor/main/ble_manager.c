#include "ble_manager.h"

#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "wearable_gatt.h"

#define BLE_MANAGER_DEFAULT_DEVICE_NAME "HeatStrokeWearable"
#define BLE_MANAGER_INVALID_CONN_HANDLE 0xFFFFU
#define BLE_MANAGER_HCI_ERR_BASE 0x0200

#ifndef BLE_MANAGER_ENABLE_VERBOSE_LOG
#define BLE_MANAGER_ENABLE_VERBOSE_LOG 0
#endif

#ifndef BLE_MANAGER_NIMBLE_LOG_LEVEL
#define BLE_MANAGER_NIMBLE_LOG_LEVEL ESP_LOG_WARN
#endif

#if BLE_MANAGER_ENABLE_VERBOSE_LOG
#define BLE_MANAGER_VLOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define BLE_MANAGER_VLOGI(...) ((void)0)
#endif

typedef struct {
    bool initialized;
    bool connected;
    uint16_t conn_handle;
    uint8_t own_addr_type;
    uint32_t last_notify_ms;
    ble_manager_config_t config;
} ble_manager_state_t;

static const char *TAG = "ble_manager";
static ble_manager_state_t s_ble = {
    .initialized = false,
    .connected = false,
    .conn_handle = BLE_MANAGER_INVALID_CONN_HANDLE,
    .own_addr_type = BLE_OWN_ADDR_PUBLIC,
    .last_notify_ms = 0U,
};

static bool s_boot_health_logged = false;

static void ble_manager_start_advertising(void);
static void ble_manager_log_boot_health_once(void);
static const char *ble_manager_disconnect_reason_to_str(int reason);
static int ble_manager_gap_event(struct ble_gap_event *event, void *arg);

static void ble_manager_log_addr(uint8_t addr_type)
{
    uint8_t addr_val[6] = {0};
    int rc = ble_hs_id_copy_addr(addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_hs_id_copy_addr failed: rc=%d", rc);
        return;
    }

    BLE_MANAGER_VLOGI("BLE address: %02X:%02X:%02X:%02X:%02X:%02X (type=%u)",
             addr_val[5],
             addr_val[4],
             addr_val[3],
             addr_val[2],
             addr_val[1],
             addr_val[0],
             (unsigned)addr_type);
}

static void ble_manager_log_boot_health_once(void)
{
    if (s_boot_health_logged) {
        return;
    }

    s_boot_health_logged = true;
    ESP_LOGI(TAG,
             "BLE health: adv=%s conn=%s name=%s service=%s",
             ble_gap_adv_active() ? "on" : "off",
             s_ble.connected ? "connected" : "disconnected",
             ble_svc_gap_device_name(),
             WEARABLE_BLE_SERVICE_UUID_STR);
}

static const char *ble_manager_disconnect_reason_to_str(int reason)
{
    if ((reason >= BLE_MANAGER_HCI_ERR_BASE) && (reason < (BLE_MANAGER_HCI_ERR_BASE + 0x100))) {
        const int hci = reason - BLE_MANAGER_HCI_ERR_BASE;
        switch (hci) {
        case 0x08:
            return "hci_connection_timeout";
        case 0x13:
            return "hci_remote_user_terminated";
        case 0x16:
            return "hci_local_host_terminated";
        case 0x3B:
            return "hci_unacceptable_conn_params";
        default:
            return "hci_error";
        }
    }

    switch (reason) {
    case 0:
        return "success";
    default:
        return "host_error";
    }
}
static void ble_manager_on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset reason=%d", reason);
}

static void ble_manager_on_settings_written(const wearable_ble_settings_t *settings, void *user_ctx)
{
    (void)user_ctx;

    if (settings == NULL) {
        return;
    }

    s_ble.last_notify_ms = 0U;

    if (s_ble.config.settings_cb != NULL) {
        s_ble.config.settings_cb(settings, s_ble.config.user_ctx);
    }
}

static void ble_manager_on_command_written(const wearable_ble_command_t *command, void *user_ctx)
{
    (void)user_ctx;

    if (command == NULL) {
        return;
    }

    switch (command->command_id) {
    case WEARABLE_CMD_NOP:
        ESP_LOGI(TAG, "command NOP received");
        break;
    case WEARABLE_CMD_FORCE_NOTIFY:
        if (!s_ble.connected) {
            ESP_LOGI(TAG, "FORCE_NOTIFY ignored: no active connection");
            break;
        }
        {
            const esp_err_t err_live = wearable_gatt_notify_live();
            const esp_err_t err_risk = wearable_gatt_notify_risk();
            const esp_err_t err_status = wearable_gatt_notify_status();
            if ((err_live == ESP_OK) && (err_risk == ESP_OK) && (err_status == ESP_OK)) {
                s_ble.last_notify_ms = (uint32_t)esp_log_timestamp();
                ESP_LOGI(TAG, "FORCE_NOTIFY executed");
            } else {
                ESP_LOGW(TAG,
                         "FORCE_NOTIFY failed: live=%s risk=%s status=%s",
                         esp_err_to_name(err_live),
                         esp_err_to_name(err_risk),
                         esp_err_to_name(err_status));
            }
        }
        break;
    case WEARABLE_CMD_RESET_COUNTERS:
        s_ble.last_notify_ms = 0U;
        ESP_LOGI(TAG, "RESET_COUNTERS received");
        break;
    default:
        ESP_LOGW(TAG, "unsupported command id: %u", (unsigned)command->command_id);
        break;
    }

    if (s_ble.config.command_cb != NULL) {
        s_ble.config.command_cb(command, s_ble.config.user_ctx);
    }
}

static void ble_manager_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: rc=%d", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &s_ble.own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: rc=%d", rc);
        return;
    }

    ble_manager_log_addr(s_ble.own_addr_type);
    ble_manager_start_advertising();
    ble_manager_log_boot_health_once();
}

static void ble_manager_start_advertising(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields scan_rsp_fields;
    struct ble_gap_adv_params adv_params;
    const ble_uuid128_t *service_uuid = wearable_gatt_service_uuid();
    const char *name = ble_svc_gap_device_name();

    if (s_ble.connected) {
        return;
    }

    if (ble_gap_adv_active()) {
        wearable_gatt_set_advertising_state(true);
        return;
    }

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    /*
     * Keep ADV payload compact (flags + service UUID).
     * Place device name in scan response to avoid 31-byte ADV overflow.
     */
    memset(&scan_rsp_fields, 0, sizeof(scan_rsp_fields));
    scan_rsp_fields.name = (uint8_t *)name;
    scan_rsp_fields.name_len = (uint8_t)strlen(name);
    scan_rsp_fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: rc=%d", rc);
        wearable_gatt_set_advertising_state(false);
        return;
    }

    rc = ble_gap_adv_rsp_set_fields(&scan_rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: rc=%d", rc);
        wearable_gatt_set_advertising_state(false);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(
        s_ble.own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_manager_gap_event, NULL);
    if (rc == BLE_HS_EALREADY) {
        wearable_gatt_set_advertising_state(true);
        return;
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: rc=%d", rc);
        wearable_gatt_set_advertising_state(false);
        return;
    }

    wearable_gatt_set_advertising_state(true);
    ESP_LOGI(TAG,
             "advertising started: name=%s service=%s (scannable)",
             ble_svc_gap_device_name(),
             WEARABLE_BLE_SERVICE_UUID_STR);
}

static int ble_manager_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_ble.connected = true;
            s_ble.conn_handle = event->connect.conn_handle;
            s_ble.last_notify_ms = 0U;

            const uint16_t mtu = ble_att_mtu(event->connect.conn_handle);
            wearable_gatt_set_connection_state(true, mtu);
            wearable_gatt_set_advertising_state(false);

            ESP_LOGI(TAG,
                     "phone connected: conn_handle=%u mtu=%u",
                     (unsigned)event->connect.conn_handle,
                     (unsigned)mtu);
            (void)wearable_gatt_notify_status();
        } else {
            ESP_LOGW(TAG,
                     "connect failed: status=%d; restarting advertising",
                     event->connect.status);
            s_ble.connected = false;
            s_ble.conn_handle = BLE_MANAGER_INVALID_CONN_HANDLE;
            wearable_gatt_set_connection_state(false, BLE_ATT_MTU_DFLT);
            ble_manager_start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG,
                 "phone disconnected: reason=%d (%s) conn_handle=%u",
                 event->disconnect.reason,
                 ble_manager_disconnect_reason_to_str(event->disconnect.reason),
                 (unsigned)event->disconnect.conn.conn_handle);

        s_ble.connected = false;
        s_ble.conn_handle = BLE_MANAGER_INVALID_CONN_HANDLE;
        wearable_gatt_set_connection_state(false, BLE_ATT_MTU_DFLT);
        (void)wearable_gatt_notify_status();
        ble_manager_start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        BLE_MANAGER_VLOGI("subscribe changed: conn=%u attr=0x%04X notify=%u indicate=%u",
                 (unsigned)event->subscribe.conn_handle,
                 (unsigned)event->subscribe.attr_handle,
                 (unsigned)event->subscribe.cur_notify,
                 (unsigned)event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_MTU:
        BLE_MANAGER_VLOGI("mtu updated: conn=%u mtu=%u",
                 (unsigned)event->mtu.conn_handle,
                 (unsigned)event->mtu.value);
        wearable_gatt_set_connection_state(s_ble.connected, event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        wearable_gatt_set_advertising_state(false);
        BLE_MANAGER_VLOGI("advertising stopped: reason=%d", event->adv_complete.reason);
        if (!s_ble.connected) {
            BLE_MANAGER_VLOGI("advertising complete (reason=%d), restarting",
                     event->adv_complete.reason);
            ble_manager_start_advertising();
        }
        return 0;

    default:
        return 0;
    }
}

static void ble_manager_host_task(void *param)
{
    (void)param;
    BLE_MANAGER_VLOGI("NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_manager_init(const ble_manager_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ble.initialized) {
        return ESP_OK;
    }

    memset(&s_ble, 0, sizeof(s_ble));
    s_ble.conn_handle = BLE_MANAGER_INVALID_CONN_HANDLE;
    s_ble.own_addr_type = BLE_OWN_ADDR_PUBLIC;
    s_ble.config = *config;
    s_boot_health_logged = false;

    if ((s_ble.config.device_name == NULL) || (s_ble.config.device_name[0] == '\0')) {
        s_ble.config.device_name = BLE_MANAGER_DEFAULT_DEVICE_NAME;
    }

    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Reduce NimBLE internal serial chatter unless explicitly overridden. */
    esp_log_level_set("NimBLE", BLE_MANAGER_NIMBLE_LOG_LEVEL);
    esp_log_level_set("NimBLE_BLE", BLE_MANAGER_NIMBLE_LOG_LEVEL);

    ble_hs_cfg.reset_cb = ble_manager_on_reset;
    ble_hs_cfg.sync_cb = ble_manager_on_sync;
    /* Prototype mode: no pairing/bonding. Production should require encrypted/authenticated access. */

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_svc_gap_device_name_set(s_ble.config.device_name);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_svc_gap_device_name_set failed: rc=%d", rc);
        return ESP_FAIL;
    }

    uint16_t initial_notify_interval_ms = s_ble.config.notify_interval_ms;
    if (initial_notify_interval_ms == 0U) {
        initial_notify_interval_ms = WEARABLE_NOTIFY_INTERVAL_DEFAULT_MS;
    } else if (initial_notify_interval_ms < WEARABLE_NOTIFY_INTERVAL_MIN_MS) {
        initial_notify_interval_ms = WEARABLE_NOTIFY_INTERVAL_MIN_MS;
    } else if (initial_notify_interval_ms > WEARABLE_NOTIFY_INTERVAL_MAX_MS) {
        initial_notify_interval_ms = WEARABLE_NOTIFY_INTERVAL_MAX_MS;
    }

    wearable_ble_settings_t initial_settings = {
        .version = WEARABLE_DATA_VERSION,
        .buzzer_enabled = s_ble.config.buzzer_enabled,
        .debug_log_level = s_ble.config.debug_log_level,
        .reserved = 0U,
        .notify_interval_ms = initial_notify_interval_ms,
        .body_temp_offset_c_x100 = s_ble.config.body_temp_offset_c_x100,
    };

    const wearable_gatt_callbacks_t gatt_callbacks = {
        .settings_cb = ble_manager_on_settings_written,
        .command_cb = ble_manager_on_command_written,
        .user_ctx = NULL,
    };

    err = wearable_gatt_init(&gatt_callbacks, &initial_settings);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wearable_gatt_init failed: %s", esp_err_to_name(err));
        return err;
    }

    rc = ble_gatts_count_cfg(wearable_gatt_service_defs());
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: rc=%d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(wearable_gatt_service_defs());
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: rc=%d", rc);
        return ESP_FAIL;
    }

    nimble_port_freertos_init(ble_manager_host_task);

    s_ble.initialized = true;
    wearable_gatt_set_connection_state(false, BLE_ATT_MTU_DFLT);
    wearable_gatt_set_advertising_state(false);

    ESP_LOGI(TAG,
             "BLE initialized: name=%s notify_interval=%u ms",
             s_ble.config.device_name,
             (unsigned)initial_settings.notify_interval_ms);
    ESP_LOGI(TAG, "BLE init complete");
    return ESP_OK;
}

esp_err_t ble_manager_publish_telemetry(const wearable_telemetry_t *telemetry)
{
    if (!s_ble.initialized || (telemetry == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }

    wearable_gatt_update_telemetry(telemetry);

    if (!s_ble.connected) {
        return ESP_OK;
    }

    const uint32_t now_ms = (uint32_t)esp_log_timestamp();
    const uint32_t notify_interval_ms = wearable_gatt_get_notify_interval_ms();
    if ((now_ms - s_ble.last_notify_ms) < notify_interval_ms) {
        return ESP_OK;
    }

    esp_err_t err_live = wearable_gatt_notify_live();
    esp_err_t err_risk = wearable_gatt_notify_risk();
    esp_err_t err_status = wearable_gatt_notify_status();

    if ((err_live != ESP_OK) || (err_risk != ESP_OK) || (err_status != ESP_OK)) {
        ESP_LOGW(TAG,
                 "notify failed: live=%s risk=%s status=%s",
                 esp_err_to_name(err_live),
                 esp_err_to_name(err_risk),
                 esp_err_to_name(err_status));
        return ESP_FAIL;
    }

    s_ble.last_notify_ms = now_ms;
    return ESP_OK;
}

bool ble_manager_is_connected(void)
{
    return s_ble.connected;
}

wearable_ble_settings_t ble_manager_get_settings(void)
{
    return wearable_gatt_get_settings();
}
