#ifndef DATA_MODEL_H
#define DATA_MODEL_H

#include <stdbool.h>
#include <stdint.h>

#define WEARABLE_DATA_VERSION 1U
#define WEARABLE_INVALID_I16 ((int16_t)0x8000)
#define WEARABLE_INVALID_U16 ((uint16_t)0xFFFF)
#define WEARABLE_INVALID_U32 ((uint32_t)0xFFFFFFFF)
#define WEARABLE_NOTIFY_INTERVAL_MIN_MS 200U
#define WEARABLE_NOTIFY_INTERVAL_MAX_MS 10000U
#define WEARABLE_NOTIFY_INTERVAL_DEFAULT_MS 1000U

typedef enum {
    WEARABLE_RISK_NORMAL = 0,
    WEARABLE_RISK_MODERATE = 1,
    WEARABLE_RISK_HIGH = 2,
    WEARABLE_RISK_VERY_HIGH = 3,
} wearable_risk_category_t;

typedef enum {
    WEARABLE_BUZZER_OFF = 0,
    WEARABLE_BUZZER_LONG_BEEP = 1,
    WEARABLE_BUZZER_SHORT_BEEP = 2,
} wearable_buzzer_state_t;

typedef enum {
    WEARABLE_CONTACT_UNKNOWN = 0,
    WEARABLE_CONTACT_NONE = 1,
    WEARABLE_CONTACT_WEAK = 2,
    WEARABLE_CONTACT_VALID = 3,
} wearable_contact_state_t;

typedef enum {
    WEARABLE_INVALID_NONE = 0,
    WEARABLE_INVALID_NO_CONTACT = 1,
    WEARABLE_INVALID_WEAK_CONTACT = 2,
    WEARABLE_INVALID_INSUFFICIENT_SAMPLES = 3,
    WEARABLE_INVALID_SIGNAL_WEAK = 4,
    WEARABLE_INVALID_SIGNAL_SATURATED = 5,
    WEARABLE_INVALID_SIGNAL_NOISY = 6,
    WEARABLE_INVALID_UNSTABLE_BEATS = 7,
    WEARABLE_INVALID_INVALID_RATIO = 8,
} wearable_invalid_reason_t;

typedef enum {
    WEARABLE_BODY_CONTACT_UNKNOWN = 0,
    WEARABLE_BODY_CONTACT_WEAK = 1,
    WEARABLE_BODY_CONTACT_VALID = 2,
} wearable_body_contact_quality_t;

typedef enum {
    WEARABLE_VALID_RAW_BODY_TEMP = (1U << 0),
    WEARABLE_VALID_FILTERED_BODY_TEMP = (1U << 1),
    WEARABLE_VALID_EST_BODY_TEMP = (1U << 2),
    WEARABLE_VALID_AMBIENT_TEMP = (1U << 3),
    WEARABLE_VALID_HUMIDITY = (1U << 4),
    WEARABLE_VALID_PRESSURE = (1U << 5),
    WEARABLE_VALID_HEAT_INDEX = (1U << 6),
    WEARABLE_VALID_HEART_RATE = (1U << 7),
    WEARABLE_VALID_SPO2 = (1U << 8),
    WEARABLE_VALID_HR_CONF = (1U << 9),
    WEARABLE_VALID_SPO2_CONF = (1U << 10),
} wearable_validity_flags_t;

typedef struct {
    /* Telemetry model values in native float form; packet structs carry scaled LE fields. */
    uint32_t sample_counter;
    uint32_t timestamp_ms;

    float raw_body_temp_c;
    float filtered_body_temp_c;
    float estimated_body_temp_c;

    float ambient_temp_c;
    float humidity_percent;
    float pressure_hpa;
    float heat_index_c;

    float heart_rate_bpm;
    float spo2_percent;
    float heart_rate_confidence;
    float spo2_confidence;

    wearable_contact_state_t ppg_contact_state;
    wearable_invalid_reason_t hr_invalid_reason;
    wearable_invalid_reason_t spo2_invalid_reason;
    wearable_body_contact_quality_t body_contact_quality;
    bool ppg_saturation;

    int8_t risk_heart_rate;
    int8_t risk_body_temp;
    int8_t risk_heat_index;
    int8_t total_risk;
    wearable_risk_category_t risk_category;
    wearable_buzzer_state_t buzzer_state;

    uint32_t validity_flags;
} wearable_telemetry_t;

typedef struct __attribute__((packed)) {
    /* Binary transport packets are packed and sent in little-endian byte order. */
    uint8_t version;
    uint8_t connected;
    uint16_t mtu;
    uint32_t uptime_ms;
    uint32_t sample_counter;
    uint16_t notify_interval_ms;
    uint8_t adv_active;
    uint8_t reserved;
} wearable_device_status_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t ppg_contact_state;
    uint8_t hr_invalid_reason;
    uint8_t spo2_invalid_reason;
    uint8_t body_contact_quality;
    uint8_t risk_category;
    uint8_t buzzer_state;
    uint8_t ppg_saturation;
    int8_t risk_heart_rate;
    int8_t risk_body_temp;
    int8_t risk_heat_index;
    int8_t total_risk;
    uint32_t validity_flags;
} wearable_risk_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t reserved0;
    uint16_t reserved1;
    uint32_t sample_counter;
    uint32_t timestamp_ms;
    int16_t raw_body_temp_c_x100;
    int16_t filtered_body_temp_c_x100;
    int16_t estimated_body_temp_c_x100;
    int16_t ambient_temp_c_x100;
    uint16_t humidity_percent_x100;
    uint16_t pressure_hpa_x10;
    int16_t heat_index_c_x100;
    uint16_t heart_rate_bpm_x10;
    uint16_t spo2_percent_x10;
    uint8_t heart_rate_conf_pct;
    uint8_t spo2_conf_pct;
    uint8_t ppg_contact_state;
    uint8_t hr_invalid_reason;
    uint8_t spo2_invalid_reason;
    uint8_t body_contact_quality;
    uint8_t risk_heart_rate;
    uint8_t risk_body_temp;
    uint8_t risk_heat_index;
    uint8_t total_risk;
    uint8_t risk_category;
    uint8_t buzzer_state;
    uint8_t ppg_saturation;
    uint8_t reserved2;
    uint32_t validity_flags;
} wearable_live_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t buzzer_enabled;
    uint8_t debug_log_level;
    uint8_t reserved;
    uint16_t notify_interval_ms;
    int16_t body_temp_offset_c_x100;
} wearable_ble_settings_t;

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t command_id;
    int16_t value_i16;
} wearable_ble_command_t;

typedef enum {
    WEARABLE_CMD_NOP = 0,
    WEARABLE_CMD_FORCE_NOTIFY = 1,
    WEARABLE_CMD_RESET_COUNTERS = 2,
} wearable_command_id_t;

/* Protocol layout guards to keep packet ABI stable for mobile decoding. */
_Static_assert(sizeof(wearable_device_status_packet_t) == 16U, "device_status packet size mismatch");
_Static_assert(sizeof(wearable_live_packet_t) == 48U, "live packet size mismatch");
_Static_assert(sizeof(wearable_risk_packet_t) == 16U, "risk packet size mismatch");
_Static_assert(sizeof(wearable_ble_settings_t) == 8U, "settings packet size mismatch");
_Static_assert(sizeof(wearable_ble_command_t) == 4U, "command packet size mismatch");

#endif
