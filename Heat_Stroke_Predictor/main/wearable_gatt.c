#include "wearable_gatt.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "host/ble_att.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "os/os_mbuf.h"

enum {
    WEARABLE_CHR_DEVICE_STATUS = 1,
    WEARABLE_CHR_LIVE_DATA = 2,
    WEARABLE_CHR_RISK_DATA = 3,
    WEARABLE_CHR_SETTINGS = 4,
    WEARABLE_CHR_COMMAND = 5,
};

static const char *TAG = "wearable_gatt";

static const ble_uuid128_t s_service_uuid =
    BLE_UUID128_INIT(0x00, 0x00, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);
static const ble_uuid128_t s_status_char_uuid =
    BLE_UUID128_INIT(0x01, 0x00, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);
static const ble_uuid128_t s_live_char_uuid =
    BLE_UUID128_INIT(0x02, 0x00, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);
static const ble_uuid128_t s_risk_char_uuid =
    BLE_UUID128_INIT(0x03, 0x00, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);
static const ble_uuid128_t s_settings_char_uuid =
    BLE_UUID128_INIT(0x04, 0x00, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);
static const ble_uuid128_t s_command_char_uuid =
    BLE_UUID128_INIT(0x05, 0x00, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static uint16_t s_status_val_handle;
static uint16_t s_live_val_handle;
static uint16_t s_risk_val_handle;
static uint16_t s_settings_val_handle;
static uint16_t s_command_val_handle;

static SemaphoreHandle_t s_lock;
static wearable_gatt_callbacks_t s_callbacks;

static wearable_telemetry_t s_latest_telemetry;
static wearable_live_packet_t s_live_packet;
static wearable_risk_packet_t s_risk_packet;
static wearable_device_status_packet_t s_status_packet;
static wearable_ble_settings_t s_settings;

static bool s_connected;
static bool s_adv_active;
static uint16_t s_mtu;

static bool wearable_notify_interval_is_valid(uint16_t interval_ms)
{
    return (interval_ms >= WEARABLE_NOTIFY_INTERVAL_MIN_MS) &&
           (interval_ms <= WEARABLE_NOTIFY_INTERVAL_MAX_MS);
}

static uint16_t wearable_clamp_notify_interval(uint16_t interval_ms)
{
    if (interval_ms < WEARABLE_NOTIFY_INTERVAL_MIN_MS) {
        return WEARABLE_NOTIFY_INTERVAL_MIN_MS;
    }
    if (interval_ms > WEARABLE_NOTIFY_INTERVAL_MAX_MS) {
        return WEARABLE_NOTIFY_INTERVAL_MAX_MS;
    }
    return interval_ms;
}

static int16_t wearable_pack_i16(float value, float scale, bool valid)
{
    if (!valid || !isfinite(value)) {
        return WEARABLE_INVALID_I16;
    }

    float scaled = value * scale;
    if (scaled > 32767.0f) {
        scaled = 32767.0f;
    }
    if (scaled < -32768.0f) {
        scaled = -32768.0f;
    }
    return (int16_t)lrintf(scaled);
}

static uint16_t wearable_pack_u16(float value, float scale, bool valid)
{
    if (!valid || !isfinite(value) || value < 0.0f) {
        return WEARABLE_INVALID_U16;
    }

    float scaled = value * scale;
    if (scaled > 65535.0f) {
        scaled = 65535.0f;
    }
    if (scaled < 0.0f) {
        scaled = 0.0f;
    }
    return (uint16_t)lrintf(scaled);
}

static uint8_t wearable_pack_confidence(float confidence)
{
    float clamped = confidence;
    if (!isfinite(clamped)) {
        clamped = 0.0f;
    }
    if (clamped < 0.0f) {
        clamped = 0.0f;
    }
    /*
     * Sensor algorithm confidence is 0.0..1.0; convert to protocol 0..100.
     * Accept already-percent values (>1.0) for forward compatibility.
     */
    if (clamped > 1.0f) {
        if (clamped > 100.0f) {
            clamped = 100.0f;
        }
        return (uint8_t)lrintf(clamped);
    }
    return (uint8_t)lrintf(clamped * 100.0f);
}

static uint8_t wearable_pack_confidence_if_valid(float confidence, uint32_t validity_flags, uint32_t flag_mask)
{
    if ((validity_flags & flag_mask) == 0U) {
        return 0U;
    }
    return wearable_pack_confidence(confidence);
}

static uint8_t wearable_pack_non_negative_u8_from_i8(int8_t value)
{
    if (value < 0) {
        return 0U;
    }
    return (uint8_t)value;
}

static bool wearable_command_id_is_supported(uint8_t command_id)
{
    return (command_id == WEARABLE_CMD_NOP) || (command_id == WEARABLE_CMD_FORCE_NOTIFY) ||
           (command_id == WEARABLE_CMD_RESET_COUNTERS);
}

static void wearable_refresh_status_locked(void)
{
    s_status_packet.version = WEARABLE_DATA_VERSION;
    s_status_packet.connected = s_connected ? 1U : 0U;
    s_status_packet.mtu = s_mtu;
    s_status_packet.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    s_status_packet.sample_counter = s_live_packet.sample_counter;
    s_status_packet.notify_interval_ms = s_settings.notify_interval_ms;
    s_status_packet.adv_active = s_adv_active ? 1U : 0U;
    s_status_packet.reserved = 0U;
}

static void wearable_pack_live_locked(const wearable_telemetry_t *t)
{
    s_live_packet.version = WEARABLE_DATA_VERSION;
    s_live_packet.reserved0 = 0U;
    s_live_packet.reserved1 = 0U;
    s_live_packet.sample_counter = t->sample_counter;
    s_live_packet.timestamp_ms = t->timestamp_ms;

    s_live_packet.raw_body_temp_c_x100 = wearable_pack_i16(
        t->raw_body_temp_c, 100.0f, (t->validity_flags & WEARABLE_VALID_RAW_BODY_TEMP) != 0U);
    s_live_packet.filtered_body_temp_c_x100 = wearable_pack_i16(
        t->filtered_body_temp_c, 100.0f, (t->validity_flags & WEARABLE_VALID_FILTERED_BODY_TEMP) != 0U);
    s_live_packet.estimated_body_temp_c_x100 = wearable_pack_i16(
        t->estimated_body_temp_c, 100.0f, (t->validity_flags & WEARABLE_VALID_EST_BODY_TEMP) != 0U);
    s_live_packet.ambient_temp_c_x100 = wearable_pack_i16(
        t->ambient_temp_c, 100.0f, (t->validity_flags & WEARABLE_VALID_AMBIENT_TEMP) != 0U);

    s_live_packet.humidity_percent_x100 = wearable_pack_u16(
        t->humidity_percent, 100.0f, (t->validity_flags & WEARABLE_VALID_HUMIDITY) != 0U);
    s_live_packet.pressure_hpa_x10 = wearable_pack_u16(
        t->pressure_hpa, 10.0f, (t->validity_flags & WEARABLE_VALID_PRESSURE) != 0U);
    s_live_packet.heat_index_c_x100 = wearable_pack_i16(
        t->heat_index_c, 100.0f, (t->validity_flags & WEARABLE_VALID_HEAT_INDEX) != 0U);

    s_live_packet.heart_rate_bpm_x10 = wearable_pack_u16(
        t->heart_rate_bpm, 10.0f, (t->validity_flags & WEARABLE_VALID_HEART_RATE) != 0U);
    s_live_packet.spo2_percent_x10 = wearable_pack_u16(
        t->spo2_percent, 10.0f, (t->validity_flags & WEARABLE_VALID_SPO2) != 0U);

    s_live_packet.heart_rate_conf_pct =
        wearable_pack_confidence_if_valid(t->heart_rate_confidence, t->validity_flags, WEARABLE_VALID_HR_CONF);
    s_live_packet.spo2_conf_pct =
        wearable_pack_confidence_if_valid(t->spo2_confidence, t->validity_flags, WEARABLE_VALID_SPO2_CONF);

    s_live_packet.ppg_contact_state = (uint8_t)t->ppg_contact_state;
    s_live_packet.hr_invalid_reason = (uint8_t)t->hr_invalid_reason;
    s_live_packet.spo2_invalid_reason = (uint8_t)t->spo2_invalid_reason;
    s_live_packet.body_contact_quality = (uint8_t)t->body_contact_quality;

    s_live_packet.risk_heart_rate = wearable_pack_non_negative_u8_from_i8(t->risk_heart_rate);
    s_live_packet.risk_body_temp = wearable_pack_non_negative_u8_from_i8(t->risk_body_temp);
    s_live_packet.risk_heat_index = wearable_pack_non_negative_u8_from_i8(t->risk_heat_index);
    s_live_packet.total_risk = wearable_pack_non_negative_u8_from_i8(t->total_risk);

    s_live_packet.risk_category = (uint8_t)t->risk_category;
    s_live_packet.buzzer_state = (uint8_t)t->buzzer_state;
    s_live_packet.ppg_saturation = t->ppg_saturation ? 1U : 0U;
    s_live_packet.reserved2 = 0U;
    s_live_packet.validity_flags = t->validity_flags;
}

static void wearable_pack_risk_locked(const wearable_telemetry_t *t)
{
    s_risk_packet.version = WEARABLE_DATA_VERSION;
    s_risk_packet.ppg_contact_state = (uint8_t)t->ppg_contact_state;
    s_risk_packet.hr_invalid_reason = (uint8_t)t->hr_invalid_reason;
    s_risk_packet.spo2_invalid_reason = (uint8_t)t->spo2_invalid_reason;
    s_risk_packet.body_contact_quality = (uint8_t)t->body_contact_quality;
    s_risk_packet.risk_category = (uint8_t)t->risk_category;
    s_risk_packet.buzzer_state = (uint8_t)t->buzzer_state;
    s_risk_packet.ppg_saturation = t->ppg_saturation ? 1U : 0U;
    s_risk_packet.risk_heart_rate = t->risk_heart_rate;
    s_risk_packet.risk_body_temp = t->risk_body_temp;
    s_risk_packet.risk_heat_index = t->risk_heat_index;
    s_risk_packet.total_risk = t->total_risk;
    s_risk_packet.validity_flags = t->validity_flags;
}

static int wearable_mbuf_append(struct os_mbuf *om, const void *data, size_t len)
{
    const int rc = os_mbuf_append(om, data, len);
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int wearable_gatt_access_cb(uint16_t conn_handle,
                                   uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt,
                                   void *arg)
{
    (void)conn_handle;
    (void)attr_handle;

    const uintptr_t chr_id = (uintptr_t)arg;
    int rc = 0;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if ((s_lock == NULL) || (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE)) {
            return BLE_ATT_ERR_UNLIKELY;
        }

        if (chr_id == WEARABLE_CHR_DEVICE_STATUS) {
            wearable_refresh_status_locked();
            rc = wearable_mbuf_append(ctxt->om, &s_status_packet, sizeof(s_status_packet));
        } else if (chr_id == WEARABLE_CHR_LIVE_DATA) {
            rc = wearable_mbuf_append(ctxt->om, &s_live_packet, sizeof(s_live_packet));
        } else if (chr_id == WEARABLE_CHR_RISK_DATA) {
            rc = wearable_mbuf_append(ctxt->om, &s_risk_packet, sizeof(s_risk_packet));
        } else if (chr_id == WEARABLE_CHR_SETTINGS) {
            rc = wearable_mbuf_append(ctxt->om, &s_settings, sizeof(s_settings));
        } else {
            rc = BLE_ATT_ERR_UNLIKELY;
        }

        xSemaphoreGive(s_lock);
        return rc;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (chr_id == WEARABLE_CHR_SETTINGS) {
            wearable_ble_settings_t incoming = {0};
            uint16_t out_len = 0;

            rc = ble_hs_mbuf_to_flat(ctxt->om, &incoming, sizeof(incoming), &out_len);
            if ((rc != 0) || (out_len != sizeof(incoming))) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            if (incoming.version != WEARABLE_DATA_VERSION) {
                ESP_LOGW(TAG,
                         "settings write rejected: unsupported version=%u (expected=%u)",
                         (unsigned)incoming.version,
                         (unsigned)WEARABLE_DATA_VERSION);
                return BLE_ATT_ERR_UNLIKELY;
            }

            if (!wearable_notify_interval_is_valid(incoming.notify_interval_ms)) {
                ESP_LOGW(TAG,
                         "settings write rejected: notify_interval_ms=%u out of range [%u..%u]",
                         (unsigned)incoming.notify_interval_ms,
                         (unsigned)WEARABLE_NOTIFY_INTERVAL_MIN_MS,
                         (unsigned)WEARABLE_NOTIFY_INTERVAL_MAX_MS);
                return BLE_ATT_ERR_UNLIKELY;
            }
            incoming.reserved = 0U;

            if ((s_lock == NULL) || (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE)) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            s_settings = incoming;
            wearable_refresh_status_locked();
            xSemaphoreGive(s_lock);

            ESP_LOGI(TAG,
                     "settings updated: notify=%u ms buzzer=%u temp_offset=%d(0.01C) debug=%u",
                     (unsigned)incoming.notify_interval_ms,
                     (unsigned)incoming.buzzer_enabled,
                     (int)incoming.body_temp_offset_c_x100,
                     (unsigned)incoming.debug_log_level);

            if (s_callbacks.settings_cb != NULL) {
                s_callbacks.settings_cb(&incoming, s_callbacks.user_ctx);
            }

            return 0;
        }

        if (chr_id == WEARABLE_CHR_COMMAND) {
            wearable_ble_command_t command = {0};
            uint16_t out_len = 0;

            rc = ble_hs_mbuf_to_flat(ctxt->om, &command, sizeof(command), &out_len);
            if ((rc != 0) || (out_len != sizeof(command))) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            if (command.version != WEARABLE_DATA_VERSION) {
                ESP_LOGW(TAG,
                         "command write rejected: unsupported version=%u (expected=%u)",
                         (unsigned)command.version,
                         (unsigned)WEARABLE_DATA_VERSION);
                return BLE_ATT_ERR_UNLIKELY;
            }

            if (!wearable_command_id_is_supported(command.command_id)) {
                ESP_LOGW(TAG,
                         "command write rejected: unsupported command_id=%u",
                         (unsigned)command.command_id);
                return BLE_ATT_ERR_UNLIKELY;
            }

            ESP_LOGI(TAG,
                     "command received: cmd=%u value=%d",
                     (unsigned)command.command_id,
                     (int)command.value_i16);

            if (s_callbacks.command_cb != NULL) {
                s_callbacks.command_cb(&command, s_callbacks.user_ctx);
            }

            return 0;
        }

        return BLE_ATT_ERR_UNLIKELY;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_chr_def s_wearable_characteristics[] = {
    {
        .uuid = &s_status_char_uuid.u,
        .access_cb = wearable_gatt_access_cb,
        .arg = (void *)(uintptr_t)WEARABLE_CHR_DEVICE_STATUS,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_status_val_handle,
    },
    {
        .uuid = &s_live_char_uuid.u,
        .access_cb = wearable_gatt_access_cb,
        .arg = (void *)(uintptr_t)WEARABLE_CHR_LIVE_DATA,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_live_val_handle,
    },
    {
        .uuid = &s_risk_char_uuid.u,
        .access_cb = wearable_gatt_access_cb,
        .arg = (void *)(uintptr_t)WEARABLE_CHR_RISK_DATA,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_risk_val_handle,
    },
    {
        .uuid = &s_settings_char_uuid.u,
        .access_cb = wearable_gatt_access_cb,
        .arg = (void *)(uintptr_t)WEARABLE_CHR_SETTINGS,
        /* Prototype mode: open writes. Production should require encrypted/authenticated writes. */
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
        .val_handle = &s_settings_val_handle,
    },
    {
        .uuid = &s_command_char_uuid.u,
        .access_cb = wearable_gatt_access_cb,
        .arg = (void *)(uintptr_t)WEARABLE_CHR_COMMAND,
        /* Prototype mode: open writes. Production should require encrypted/authenticated writes. */
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .val_handle = &s_command_val_handle,
    },
    {0},
};

static const struct ble_gatt_svc_def s_wearable_service[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = s_wearable_characteristics,
    },
    {0},
};

esp_err_t wearable_gatt_init(const wearable_gatt_callbacks_t *callbacks,
                             const wearable_ble_settings_t *initial_settings)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    memset(&s_latest_telemetry, 0, sizeof(s_latest_telemetry));
    memset(&s_live_packet, 0, sizeof(s_live_packet));
    memset(&s_risk_packet, 0, sizeof(s_risk_packet));
    memset(&s_status_packet, 0, sizeof(s_status_packet));
    memset(&s_settings, 0, sizeof(s_settings));

    s_connected = false;
    s_adv_active = false;
    s_mtu = BLE_ATT_MTU_DFLT;

    if (callbacks != NULL) {
        s_callbacks = *callbacks;
    } else {
        memset(&s_callbacks, 0, sizeof(s_callbacks));
    }

    if (initial_settings != NULL) {
        s_settings = *initial_settings;
    }

    s_settings.version = WEARABLE_DATA_VERSION;
    s_settings.reserved = 0U;
    s_settings.notify_interval_ms =
        (s_settings.notify_interval_ms == 0U) ? WEARABLE_NOTIFY_INTERVAL_DEFAULT_MS :
                                                 wearable_clamp_notify_interval(s_settings.notify_interval_ms);

    wearable_refresh_status_locked();
    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG,
             "GATT state initialized (notify=%u ms)",
             (unsigned)s_settings.notify_interval_ms);
    return ESP_OK;
}

const struct ble_gatt_svc_def *wearable_gatt_service_defs(void)
{
    return s_wearable_service;
}

const ble_uuid128_t *wearable_gatt_service_uuid(void)
{
    return &s_service_uuid;
}

void wearable_gatt_set_connection_state(bool connected, uint16_t mtu)
{
    if ((s_lock == NULL) || (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE)) {
        return;
    }

    s_connected = connected;
    s_mtu = (mtu == 0U) ? BLE_ATT_MTU_DFLT : mtu;
    wearable_refresh_status_locked();

    xSemaphoreGive(s_lock);
}

void wearable_gatt_set_advertising_state(bool active)
{
    if ((s_lock == NULL) || (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE)) {
        return;
    }

    s_adv_active = active;
    wearable_refresh_status_locked();

    xSemaphoreGive(s_lock);
}

uint16_t wearable_gatt_get_notify_interval_ms(void)
{
    uint16_t value = WEARABLE_NOTIFY_INTERVAL_DEFAULT_MS;

    if ((s_lock == NULL) || (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE)) {
        return value;
    }

    value = s_settings.notify_interval_ms;
    xSemaphoreGive(s_lock);
    return value;
}

wearable_ble_settings_t wearable_gatt_get_settings(void)
{
    wearable_ble_settings_t copy;
    memset(&copy, 0, sizeof(copy));
    copy.version = WEARABLE_DATA_VERSION;
    copy.reserved = 0U;
    copy.notify_interval_ms = WEARABLE_NOTIFY_INTERVAL_DEFAULT_MS;

    if ((s_lock == NULL) || (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE)) {
        return copy;
    }

    copy = s_settings;
    xSemaphoreGive(s_lock);

    return copy;
}

void wearable_gatt_update_telemetry(const wearable_telemetry_t *telemetry)
{
    if ((telemetry == NULL) || (s_lock == NULL)) {
        return;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }

    s_latest_telemetry = *telemetry;
    wearable_pack_live_locked(telemetry);
    wearable_pack_risk_locked(telemetry);
    wearable_refresh_status_locked();

    xSemaphoreGive(s_lock);
}

esp_err_t wearable_gatt_notify_live(void)
{
    if (s_live_val_handle == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_gatts_chr_updated(s_live_val_handle);
    return ESP_OK;
}

esp_err_t wearable_gatt_notify_risk(void)
{
    if (s_risk_val_handle == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_gatts_chr_updated(s_risk_val_handle);
    return ESP_OK;
}

esp_err_t wearable_gatt_notify_status(void)
{
    if (s_status_val_handle == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    ble_gatts_chr_updated(s_status_val_handle);
    return ESP_OK;
}
