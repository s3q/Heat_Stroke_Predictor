#include "app_logic.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ble_manager.h"
#include "bme280.h"
#include "data_model.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_manager.h"
#include "max30102.h"
#include "max30205.h"
#include "sdkconfig.h"

/*
 * Upper-arm wearable estimation model for MAX30205 skin-contact temperature.
 * This is an estimated body-temperature signal for prototyping and is not a
 * clinical core-body measurement. Tune coefficients using per-user reference data.
 */
#define MAX30205_CAL_MODE_OFFSET 1
#define MAX30205_CAL_MODE_LINEAR 2

#ifndef MAX30205_CAL_MODE
#define MAX30205_CAL_MODE MAX30205_CAL_MODE_OFFSET
#endif

#ifndef MAX30205_TEMP_OFFSET_C
#define MAX30205_TEMP_OFFSET_C 0.0f
#endif

#ifndef MAX30205_TEMP_SCALE
#define MAX30205_TEMP_SCALE 1.0f
#endif

#ifndef MAX30205_TEMP_BIAS_C
#define MAX30205_TEMP_BIAS_C 0.0f
#endif

#ifndef MAX30205_CONTACT_DELTA_MIN_C
#define MAX30205_CONTACT_DELTA_MIN_C 3.0f
#endif
#ifndef MAX30205_CONTACT_BODY_MIN_C
#define MAX30205_CONTACT_BODY_MIN_C 0.0f
#endif

#ifndef MAX30205_AMBIENT_DELTA_GAIN
#define MAX30205_AMBIENT_DELTA_GAIN 1.0f
#endif

#ifndef MAX30205_FILTER_WINDOW_SAMPLES
#define MAX30205_FILTER_WINDOW_SAMPLES 8U
#endif

#ifndef MAX30102_FIFO_READ_BUDGET_PER_CYCLE
#define MAX30102_FIFO_READ_BUDGET_PER_CYCLE 256
#endif

#ifndef MAX30102_MAX_READ_ERRORS_BEFORE_RESET
#define MAX30102_MAX_READ_ERRORS_BEFORE_RESET 3U
#endif

#ifndef APP_BUZZER_ACTIVE_HIGH
#define APP_BUZZER_ACTIVE_HIGH 1
#endif

#ifndef APP_STATUS_LED_ACTIVE_HIGH
#define APP_STATUS_LED_ACTIVE_HIGH 1
#endif

#ifndef APP_OUTPUT_TIMER_PERIOD_MS
#define APP_OUTPUT_TIMER_PERIOD_MS 100U
#endif

#ifndef APP_ENABLE_SENSOR_LOOP_LOG
#define APP_ENABLE_SENSOR_LOOP_LOG 1
#endif

#ifndef APP_SENSOR_LOOP_LOG_INTERVAL_MS
#define APP_SENSOR_LOOP_LOG_INTERVAL_MS 5000U
#endif
#ifndef APP_BUZZER_SHORT_ON_MS
#define APP_BUZZER_SHORT_ON_MS 120U
#endif

#ifndef APP_BUZZER_SHORT_OFF_MS
#define APP_BUZZER_SHORT_OFF_MS 180U
#endif

#ifndef APP_BUZZER_LONG_ON_MS
#define APP_BUZZER_LONG_ON_MS 650U
#endif

#ifndef APP_BUZZER_LONG_OFF_MS
#define APP_BUZZER_LONG_OFF_MS 350U
#endif

#ifndef APP_LED_MODERATE_ON_MS
#define APP_LED_MODERATE_ON_MS 500U
#endif

#ifndef APP_LED_MODERATE_OFF_MS
#define APP_LED_MODERATE_OFF_MS 500U
#endif

#ifndef APP_LED_VERY_HIGH_ON_MS
#define APP_LED_VERY_HIGH_ON_MS 125U
#endif

#ifndef APP_LED_VERY_HIGH_OFF_MS
#define APP_LED_VERY_HIGH_OFF_MS 125U
#endif

#ifndef CONFIG_APP_RISK_DEBOUNCE_CYCLES
#define CONFIG_APP_RISK_DEBOUNCE_CYCLES 3
#endif

typedef struct {
    bool ready;
    uint32_t next_retry_at_ms;
} app_sensor_slot_t;

typedef enum {
    APP_RISK_CATEGORY_NORMAL = 0,
    APP_RISK_CATEGORY_MODERATE = 1,
    APP_RISK_CATEGORY_HIGH = 2,
    APP_RISK_CATEGORY_VERY_HIGH = 3,
} app_risk_category_t;

typedef enum {
    APP_BUZZER_MODE_OFF = 0,
    APP_BUZZER_MODE_LONG_BEEP = 1,
    APP_BUZZER_MODE_SHORT_BEEP = 2,
} app_buzzer_mode_t;

/*
 * Per-cycle sensor snapshot used by app logic.
 *
 * Design intent for beginners:
 * - Keep raw channels (what sensor/algorithm directly produced).
 * - Keep processed channels (filtered/calibrated/estimated values).
 * - Track validity flags so logs and BLE can print NA safely.
 * - Keep risk and alert state in the same snapshot for one log line.
 */
typedef struct {
    float body_temp_c;
    bool body_temp_valid;
    float raw_body_temp_c;
    bool raw_body_temp_valid;
    float filtered_body_temp_c;
    bool filtered_body_temp_valid;
    float est_body_temp_c;
    bool est_body_temp_valid;
    float body_ambient_delta_c;
    bool body_ambient_delta_valid;
    bool contact_quality_known;
    bool contact_quality_valid;
    uint32_t red_raw;
    uint32_t ir_raw;
    bool ppg_raw_valid;
    float red_filtered;
    float ir_filtered;
    bool ppg_filtered_valid;
    float red_dc;
    float ir_dc;
    float red_ac_rms;
    float ir_ac_rms;
    bool ppg_quality_valid;
    max30102_contact_quality_t ppg_contact_quality;
    bool ppg_signal_saturated;
    float heart_rate_bpm;
    bool heart_rate_valid;
    float heart_rate_confidence;
    max30102_reason_t hr_reason;
    float spo2_percent;
    bool spo2_valid;
    float spo2_confidence;
    max30102_reason_t spo2_reason;
    float heat_index_c;
    bool heat_index_valid;
    int heart_rate_risk;
    bool heart_rate_risk_applied;
    int body_temp_risk;
    bool body_temp_risk_applied;
    int heat_index_risk;
    bool heat_index_risk_applied;
    int total_risk;
    app_risk_category_t raw_risk_category;
    app_risk_category_t stable_risk_category;
    app_buzzer_mode_t buzzer_mode;
    float env_temp_c;
    bool env_temp_valid;
    float humidity_percent;
    bool humidity_valid;
    float pressure_hpa;
    bool pressure_valid;
} app_sensor_readings_t;

/*
 * Persistent application runtime context.
 *
 * This struct stores hardware handles, filter memory, algorithm state,
 * retry timers, and alert/telemetry state that must survive across loops.
 */
typedef struct {
    max30205_t max30205;
    max30102_t max30102;
    bme280_t bme280;
    max30102_algo_state_t ppg_algo;
    max30102_processed_t ppg_processed;
    app_sensor_slot_t max30205_slot;
    app_sensor_slot_t max30102_slot;
    app_sensor_slot_t bme280_slot;
    float max30205_ma_buffer[MAX30205_FILTER_WINDOW_SAMPLES];
    uint8_t max30205_ma_count;
    uint8_t max30205_ma_head;
    float max30205_ma_sum;
    uint8_t max30102_read_error_streak;
    app_risk_category_t pending_risk_category;
    uint8_t pending_risk_category_count;
    app_risk_category_t stable_risk_category;
    app_buzzer_mode_t buzzer_mode;
    bool risk_state_initialized;
    bool output_timer_started;
    esp_timer_handle_t output_timer;
    float ble_runtime_temp_offset_c;
    bool ble_buzzer_enabled;
    uint32_t sample_counter;
    bool started;
} app_context_t;

static const char *TAG = "app_logic";
static TaskHandle_t s_task_handle;
static app_context_t s_app_ctx;

// Heart rate scoring per Silawarawet et al. 2025 Table 1
static int heartRateRisk(float hr_bpm)
{
    if (hr_bpm >= 189.0f) {
        return 2;
    }
    if (hr_bpm >= 100.0f) {
        return 1;
    }
    return 0;
}

// Body temperature scoring per Silawarawet et al. 2025 Table 1
static int bodyTempRisk(float body_temp_c)
{
    if (body_temp_c >= 39.1f) {
        return 3;
    }
    if (body_temp_c >= 38.1f) {
        return 2;
    }
    if (body_temp_c >= 37.3f) {
        return 1;
    }
    return 0;
}

// Heat-index equation used in Silawarawet et al. 2025 (from Awasthi et al. 2021).
// Inputs: ambient_temp_c in degree Celsius, rh_percent in percent (0..100).
// Formula is computed in degree Fahrenheit, then converted back to Celsius.
static float computeHeatIndexC(float ambient_temp_c, float rh_percent)
{
    const float rh = fminf(fmaxf(rh_percent, 0.0f), 100.0f);
    const float t_f = (ambient_temp_c * 9.0f / 5.0f) + 32.0f;

    const float hi_f = -42.379f + (2.04901523f * t_f) + (10.14333127f * rh) - (0.22475541f * t_f * rh) -
                       (0.00683783f * t_f * t_f) - (0.05481717f * rh * rh) +
                       (0.00122874f * t_f * t_f * rh) + (0.00085282f * t_f * rh * rh) -
                       (0.00000199f * t_f * t_f * rh * rh);

    return (hi_f - 32.0f) * 5.0f / 9.0f;
}

// Heat-index band scoring per Silawarawet et al. 2025 Table 3
static int heatIndexRisk(float heat_index_c)
{
    /*
     * Paper Table 3 provides risk ranges per band (0-2, 3-5, 5-6, 7-8).
     * For deterministic embedded behavior, use the lower bound score of each band.
     */
    if (heat_index_c >= 55.0f) {
        return 7;
    }
    if (heat_index_c >= 42.0f) {
        return 5;
    }
    if (heat_index_c >= 33.0f) {
        return 3;
    }
    return 0;
}

// Final risk category per Silawarawet et al. 2025 Table 4
static int totalRiskCategory(int total_risk)
{
    if (total_risk <= 0) {
        return APP_RISK_CATEGORY_NORMAL;
    }
    if (total_risk <= 4) {
        return APP_RISK_CATEGORY_MODERATE;
    }
    if (total_risk <= 6) {
        return APP_RISK_CATEGORY_HIGH;
    }
    return APP_RISK_CATEGORY_VERY_HIGH;
}

static app_buzzer_mode_t app_logic_buzzer_mode_from_category(app_risk_category_t category)
{
    // Buzzer behavior per Silawarawet et al. 2025 notification flowchart
    if (category == APP_RISK_CATEGORY_VERY_HIGH) {
        return APP_BUZZER_MODE_SHORT_BEEP;
    }
    if (category == APP_RISK_CATEGORY_HIGH) {
        return APP_BUZZER_MODE_LONG_BEEP;
    }
    return APP_BUZZER_MODE_OFF;
}

static const char *app_logic_risk_category_to_string(app_risk_category_t category)
{
    switch (category) {
    case APP_RISK_CATEGORY_NORMAL:
        return "NORMAL";
    case APP_RISK_CATEGORY_MODERATE:
        return "MODERATE";
    case APP_RISK_CATEGORY_HIGH:
        return "HIGH";
    case APP_RISK_CATEGORY_VERY_HIGH:
        return "VERY_HIGH";
    default:
        return "UNKNOWN";
    }
}

static const char *app_logic_buzzer_mode_to_string(app_buzzer_mode_t mode)
{
    switch (mode) {
    case APP_BUZZER_MODE_OFF:
        return "OFF";
    case APP_BUZZER_MODE_LONG_BEEP:
        return "LONG_BEEP";
    case APP_BUZZER_MODE_SHORT_BEEP:
        return "SHORT_BEEP";
    default:
        return "UNKNOWN";
    }
}

static wearable_contact_state_t app_logic_map_ppg_contact(max30102_contact_quality_t contact_quality)
{
    switch (contact_quality) {
    case MAX30102_CONTACT_VALID:
        return WEARABLE_CONTACT_VALID;
    case MAX30102_CONTACT_WEAK:
        return WEARABLE_CONTACT_WEAK;
    case MAX30102_CONTACT_NO_FINGER:
    default:
        return WEARABLE_CONTACT_NONE;
    }
}

static wearable_invalid_reason_t app_logic_map_invalid_reason(max30102_reason_t reason)
{
    switch (reason) {
    case MAX30102_REASON_NONE:
        return WEARABLE_INVALID_NONE;
    case MAX30102_REASON_NO_CONTACT:
        return WEARABLE_INVALID_NO_CONTACT;
    case MAX30102_REASON_WEAK_CONTACT:
        return WEARABLE_INVALID_WEAK_CONTACT;
    case MAX30102_REASON_INSUFFICIENT_SAMPLES:
        return WEARABLE_INVALID_INSUFFICIENT_SAMPLES;
    case MAX30102_REASON_SIGNAL_WEAK:
        return WEARABLE_INVALID_SIGNAL_WEAK;
    case MAX30102_REASON_SIGNAL_SATURATED:
        return WEARABLE_INVALID_SIGNAL_SATURATED;
    case MAX30102_REASON_SIGNAL_NOISY:
        return WEARABLE_INVALID_SIGNAL_NOISY;
    case MAX30102_REASON_UNSTABLE_BEATS:
        return WEARABLE_INVALID_UNSTABLE_BEATS;
    case MAX30102_REASON_INVALID_RATIO:
    default:
        return WEARABLE_INVALID_INVALID_RATIO;
    }
}

static wearable_body_contact_quality_t app_logic_map_body_contact_quality(const app_sensor_readings_t *r)
{
    if (!r->contact_quality_known) {
        return WEARABLE_BODY_CONTACT_UNKNOWN;
    }
    return r->contact_quality_valid ? WEARABLE_BODY_CONTACT_VALID : WEARABLE_BODY_CONTACT_WEAK;
}

static wearable_risk_category_t app_logic_map_risk_category(app_risk_category_t category)
{
    switch (category) {
    case APP_RISK_CATEGORY_NORMAL:
        return WEARABLE_RISK_NORMAL;
    case APP_RISK_CATEGORY_MODERATE:
        return WEARABLE_RISK_MODERATE;
    case APP_RISK_CATEGORY_HIGH:
        return WEARABLE_RISK_HIGH;
    case APP_RISK_CATEGORY_VERY_HIGH:
    default:
        return WEARABLE_RISK_VERY_HIGH;
    }
}

static wearable_buzzer_state_t app_logic_map_buzzer_state(app_buzzer_mode_t mode)
{
    switch (mode) {
    case APP_BUZZER_MODE_OFF:
        return WEARABLE_BUZZER_OFF;
    case APP_BUZZER_MODE_LONG_BEEP:
        return WEARABLE_BUZZER_LONG_BEEP;
    case APP_BUZZER_MODE_SHORT_BEEP:
    default:
        return WEARABLE_BUZZER_SHORT_BEEP;
    }
}

static uint32_t app_logic_build_validity_flags(const app_sensor_readings_t *r)
{
    uint32_t flags = 0U;
    if (r->raw_body_temp_valid) {
        flags |= WEARABLE_VALID_RAW_BODY_TEMP;
    }
    if (r->filtered_body_temp_valid) {
        flags |= WEARABLE_VALID_FILTERED_BODY_TEMP;
    }
    if (r->est_body_temp_valid) {
        flags |= WEARABLE_VALID_EST_BODY_TEMP;
    }
    if (r->env_temp_valid) {
        flags |= WEARABLE_VALID_AMBIENT_TEMP;
    }
    if (r->humidity_valid) {
        flags |= WEARABLE_VALID_HUMIDITY;
    }
    if (r->pressure_valid) {
        flags |= WEARABLE_VALID_PRESSURE;
    }
    if (r->heat_index_valid) {
        flags |= WEARABLE_VALID_HEAT_INDEX;
    }
    if (r->heart_rate_valid) {
        flags |= WEARABLE_VALID_HEART_RATE;
    }
    if (r->spo2_valid) {
        flags |= WEARABLE_VALID_SPO2;
    }
    if (r->ppg_quality_valid) {
        flags |= WEARABLE_VALID_HR_CONF | WEARABLE_VALID_SPO2_CONF;
    }
    return flags;
}

static esp_log_level_t app_logic_ble_log_level(uint8_t debug_level)
{
    switch (debug_level) {
    case 0:
        return ESP_LOG_NONE;
    case 1:
        return ESP_LOG_ERROR;
    case 2:
        return ESP_LOG_WARN;
    case 4:
        return ESP_LOG_DEBUG;
    case 5:
        return ESP_LOG_VERBOSE;
    case 3:
    default:
        return ESP_LOG_INFO;
    }
}

static void app_logic_ble_settings_callback(const wearable_ble_settings_t *settings, void *user_ctx)
{
    app_context_t *ctx = (app_context_t *)user_ctx;
    if ((ctx == NULL) || (settings == NULL)) {
        return;
    }

    ctx->ble_buzzer_enabled = (settings->buzzer_enabled != 0U);
    ctx->ble_runtime_temp_offset_c = ((float)settings->body_temp_offset_c_x100) / 100.0f;

    const esp_log_level_t level = app_logic_ble_log_level(settings->debug_log_level);
    esp_log_level_set(TAG, level);
    esp_log_level_set("ble_manager", level);
    esp_log_level_set("wearable_gatt", level);

    ESP_LOGI(TAG,
             "BLE settings applied: notify=%u ms buzzer=%s temp_offset=%.2fC debug=%u",
             (unsigned)settings->notify_interval_ms,
             ctx->ble_buzzer_enabled ? "on" : "off",
             ctx->ble_runtime_temp_offset_c,
             (unsigned)settings->debug_log_level);
}

static void app_logic_ble_command_callback(const wearable_ble_command_t *command, void *user_ctx)
{
    app_context_t *ctx = (app_context_t *)user_ctx;
    if ((ctx == NULL) || (command == NULL)) {
        return;
    }

    switch (command->command_id) {
    case WEARABLE_CMD_NOP:
        ESP_LOGI(TAG, "BLE command: NOP");
        break;
    case WEARABLE_CMD_FORCE_NOTIFY:
        ESP_LOGI(TAG, "BLE command: FORCE_NOTIFY");
        break;
    case WEARABLE_CMD_RESET_COUNTERS:
        ctx->sample_counter = 0U;
        ctx->pending_risk_category_count = 0U;
        ctx->risk_state_initialized = false;
        ESP_LOGI(TAG, "BLE command: counters reset");
        break;
    default:
        ESP_LOGW(TAG, "BLE command: unsupported command_id=%u", (unsigned)command->command_id);
        break;
    }
}

static void app_logic_publish_ble_telemetry(app_context_t *ctx, const app_sensor_readings_t *r)
{
#if CONFIG_APP_BLE_ENABLE
    wearable_telemetry_t telemetry;
    memset(&telemetry, 0, sizeof(telemetry));

    telemetry.sample_counter = ++ctx->sample_counter;
    telemetry.timestamp_ms = (uint32_t)esp_log_timestamp();

    telemetry.raw_body_temp_c = r->raw_body_temp_c;
    telemetry.filtered_body_temp_c = r->filtered_body_temp_c;
    telemetry.estimated_body_temp_c = r->est_body_temp_c;
    telemetry.ambient_temp_c = r->env_temp_c;
    telemetry.humidity_percent = r->humidity_percent;
    telemetry.pressure_hpa = r->pressure_hpa;
    telemetry.heat_index_c = r->heat_index_c;
    telemetry.heart_rate_bpm = r->heart_rate_bpm;
    telemetry.spo2_percent = r->spo2_percent;
    telemetry.heart_rate_confidence = r->heart_rate_confidence;
    telemetry.spo2_confidence = r->spo2_confidence;

    telemetry.ppg_contact_state = app_logic_map_ppg_contact(r->ppg_contact_quality);
    telemetry.hr_invalid_reason = app_logic_map_invalid_reason(r->hr_reason);
    telemetry.spo2_invalid_reason = app_logic_map_invalid_reason(r->spo2_reason);
    telemetry.body_contact_quality = app_logic_map_body_contact_quality(r);
    telemetry.ppg_saturation = r->ppg_signal_saturated;

    telemetry.risk_heart_rate = (int8_t)r->heart_rate_risk;
    telemetry.risk_body_temp = (int8_t)r->body_temp_risk;
    telemetry.risk_heat_index = (int8_t)r->heat_index_risk;
    telemetry.total_risk = (int8_t)r->total_risk;
    telemetry.risk_category = app_logic_map_risk_category(r->stable_risk_category);
    telemetry.buzzer_state = app_logic_map_buzzer_state(r->buzzer_mode);
    telemetry.validity_flags = app_logic_build_validity_flags(r);

    const esp_err_t err = ble_manager_publish_telemetry(&telemetry);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        ESP_LOGW(TAG, "BLE publish failed: %s", esp_err_to_name(err));
    }
#else
    (void)ctx;
    (void)r;
#endif
}

static int app_logic_gpio_level(bool on, bool active_high)
{
    return on ? (active_high ? 1 : 0) : (active_high ? 0 : 1);
}

static void app_logic_output_timer_cb(void *arg)
{
    app_context_t *ctx = (app_context_t *)arg;
    if (ctx == NULL) {
        return;
    }

    const uint32_t now_ms = (uint32_t)esp_log_timestamp();

#if CONFIG_APP_BUZZER_GPIO >= 0
    bool buzzer_on = false;
    if (ctx->buzzer_mode == APP_BUZZER_MODE_SHORT_BEEP) {
        const uint32_t period = APP_BUZZER_SHORT_ON_MS + APP_BUZZER_SHORT_OFF_MS;
        buzzer_on = ((now_ms % period) < APP_BUZZER_SHORT_ON_MS);
    } else if (ctx->buzzer_mode == APP_BUZZER_MODE_LONG_BEEP) {
        const uint32_t period = APP_BUZZER_LONG_ON_MS + APP_BUZZER_LONG_OFF_MS;
        buzzer_on = ((now_ms % period) < APP_BUZZER_LONG_ON_MS);
    }
    (void)gpio_set_level((gpio_num_t)CONFIG_APP_BUZZER_GPIO, app_logic_gpio_level(buzzer_on, APP_BUZZER_ACTIVE_HIGH != 0));
#endif

#if CONFIG_APP_STATUS_LED_GPIO >= 0
    bool led_on = false;
    if (ctx->stable_risk_category == APP_RISK_CATEGORY_MODERATE) {
        const uint32_t period = APP_LED_MODERATE_ON_MS + APP_LED_MODERATE_OFF_MS;
        led_on = ((now_ms % period) < APP_LED_MODERATE_ON_MS);
    } else if (ctx->stable_risk_category == APP_RISK_CATEGORY_HIGH) {
        led_on = true;
    } else if (ctx->stable_risk_category == APP_RISK_CATEGORY_VERY_HIGH) {
        const uint32_t period = APP_LED_VERY_HIGH_ON_MS + APP_LED_VERY_HIGH_OFF_MS;
        led_on = ((now_ms % period) < APP_LED_VERY_HIGH_ON_MS);
    }
    (void)gpio_set_level((gpio_num_t)CONFIG_APP_STATUS_LED_GPIO, app_logic_gpio_level(led_on, APP_STATUS_LED_ACTIVE_HIGH != 0));
#endif
}

static esp_err_t app_logic_output_init(app_context_t *ctx)
{
    esp_err_t err = ESP_OK;
#if CONFIG_APP_BUZZER_GPIO >= 0
    err = gpio_reset_pin((gpio_num_t)CONFIG_APP_BUZZER_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "buzzer gpio reset failed: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_set_direction((gpio_num_t)CONFIG_APP_BUZZER_GPIO, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "buzzer gpio dir failed: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_set_level((gpio_num_t)CONFIG_APP_BUZZER_GPIO, app_logic_gpio_level(false, APP_BUZZER_ACTIVE_HIGH != 0));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "buzzer gpio level failed: %s", esp_err_to_name(err));
        return err;
    }
#endif

#if CONFIG_APP_STATUS_LED_GPIO >= 0
    err = gpio_reset_pin((gpio_num_t)CONFIG_APP_STATUS_LED_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led gpio reset failed: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_set_direction((gpio_num_t)CONFIG_APP_STATUS_LED_GPIO, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led gpio dir failed: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_set_level((gpio_num_t)CONFIG_APP_STATUS_LED_GPIO, app_logic_gpio_level(false, APP_STATUS_LED_ACTIVE_HIGH != 0));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led gpio level failed: %s", esp_err_to_name(err));
        return err;
    }
#endif

#if (CONFIG_APP_BUZZER_GPIO >= 0) || (CONFIG_APP_STATUS_LED_GPIO >= 0)
    const esp_timer_create_args_t timer_args = {
        .callback = app_logic_output_timer_cb,
        .arg = ctx,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "risk_io",
    };
    err = esp_timer_create(&timer_args, &ctx->output_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "output timer create failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_timer_start_periodic(ctx->output_timer, APP_OUTPUT_TIMER_PERIOD_MS * 1000ULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "output timer start failed: %s", esp_err_to_name(err));
        (void)esp_timer_delete(ctx->output_timer);
        ctx->output_timer = NULL;
        return err;
    }
    ctx->output_timer_started = true;
#endif

    return ESP_OK;
}

static void app_logic_update_risk_state(app_context_t *ctx, app_sensor_readings_t *r)
{
    if (!ctx->risk_state_initialized) {
        ctx->risk_state_initialized = true;
        ctx->pending_risk_category = r->raw_risk_category;
        ctx->pending_risk_category_count = 0U;
        ctx->stable_risk_category = r->raw_risk_category;
    } else if (r->raw_risk_category == ctx->stable_risk_category) {
        ctx->pending_risk_category = r->raw_risk_category;
        ctx->pending_risk_category_count = 0U;
    } else {
        if (r->raw_risk_category != ctx->pending_risk_category) {
            ctx->pending_risk_category = r->raw_risk_category;
            ctx->pending_risk_category_count = 1U;
        } else if (ctx->pending_risk_category_count < UINT8_MAX) {
            ctx->pending_risk_category_count++;
        }

        if (ctx->pending_risk_category_count >= (uint8_t)CONFIG_APP_RISK_DEBOUNCE_CYCLES) {
            ctx->stable_risk_category = ctx->pending_risk_category;
            ctx->pending_risk_category_count = 0U;
        }
    }

    ctx->buzzer_mode =
        ctx->ble_buzzer_enabled ? app_logic_buzzer_mode_from_category(ctx->stable_risk_category) : APP_BUZZER_MODE_OFF;
    r->stable_risk_category = ctx->stable_risk_category;
    r->buzzer_mode = ctx->buzzer_mode;
}

static void app_logic_compute_heatstroke_risk(app_context_t *ctx, app_sensor_readings_t *r)
{
    r->heart_rate_risk = 0;
    r->heart_rate_risk_applied = false;
    r->body_temp_risk = 0;
    r->body_temp_risk_applied = false;
    r->heat_index_c = NAN;
    r->heat_index_valid = false;
    r->heat_index_risk = 0;
    r->heat_index_risk_applied = false;

    if (r->heart_rate_valid && isfinite(r->heart_rate_bpm)) {
        r->heart_rate_risk = heartRateRisk(r->heart_rate_bpm);
        r->heart_rate_risk_applied = true;
    }

    if (r->body_temp_valid && isfinite(r->body_temp_c)) {
        r->body_temp_risk = bodyTempRisk(r->body_temp_c);
        r->body_temp_risk_applied = true;
    }

    if (r->env_temp_valid && isfinite(r->env_temp_c) && r->humidity_valid && isfinite(r->humidity_percent)) {
        r->heat_index_c = computeHeatIndexC(r->env_temp_c, r->humidity_percent);
        r->heat_index_valid = true;
        r->heat_index_risk = heatIndexRisk(r->heat_index_c);
        r->heat_index_risk_applied = true;
    }

    r->total_risk = r->heart_rate_risk + r->body_temp_risk + r->heat_index_risk;
    r->raw_risk_category = (app_risk_category_t)totalRiskCategory(r->total_risk);
    app_logic_update_risk_state(ctx, r);
}

/*
 * Reset moving-average memory for MAX30205.
 *
 * Why reset is important:
 * - On sensor re-init or startup, old samples must not bias new readings.
 * - This guarantees first filtered values only use fresh data.
 */
static void app_logic_max30205_filter_reset(app_context_t *ctx)
{
    memset(ctx->max30205_ma_buffer, 0, sizeof(ctx->max30205_ma_buffer));
    ctx->max30205_ma_count = 0U;
    ctx->max30205_ma_head = 0U;
    ctx->max30205_ma_sum = 0.0f;
}

/*
 * Update MAX30205 moving average and return filtered temperature.
 *
 * Behavior:
 * - During warm-up (buffer not full), average over available samples.
 * - After warm-up, use a fixed-size sliding window.
 */
static float app_logic_max30205_filter_update(app_context_t *ctx, float raw_temp_c)
{
    // printf("Raw temp: %.2fC\n", raw_temp_c);
    if (ctx->max30205_ma_count < (uint8_t)MAX30205_FILTER_WINDOW_SAMPLES) {
        ctx->max30205_ma_buffer[ctx->max30205_ma_count] = raw_temp_c;
        ctx->max30205_ma_sum += raw_temp_c;
        ctx->max30205_ma_count++;
        return ctx->max30205_ma_sum / (float)ctx->max30205_ma_count;
    }

    const uint8_t idx = ctx->max30205_ma_head;
    ctx->max30205_ma_sum -= ctx->max30205_ma_buffer[idx];
    ctx->max30205_ma_buffer[idx] = raw_temp_c;
    ctx->max30205_ma_sum += raw_temp_c;
    ctx->max30205_ma_head = (uint8_t)((ctx->max30205_ma_head + 1U) % (uint8_t)MAX30205_FILTER_WINDOW_SAMPLES);
    return ctx->max30205_ma_sum / (float)MAX30205_FILTER_WINDOW_SAMPLES;
}

/*
 * Convert raw/filtered skin-contact temperature into estimated body channel.
 *
 * Calibration model:
 * - OFFSET mode: estimated = raw + offset (+ runtime BLE offset)
 * - LINEAR mode: estimated = scale * raw + bias (+ runtime BLE offset)
 *
 * Extra ambient term:
 * - If ambient is known and lower than skin-contact reading, optional gain can
 *   compensate part of heat loss to environment.
 *
 * Safety clamp keeps output in a practical wearable range.
 */
static float max30205_apply_calibration(app_context_t *ctx, float raw_temp_c, float ambient_temp_c)
{
    const float runtime_offset_c = (ctx != NULL) ? ctx->ble_runtime_temp_offset_c : 0.0f;
    float est_temp_c = raw_temp_c;
#if MAX30205_CAL_MODE == MAX30205_CAL_MODE_LINEAR
    est_temp_c = (MAX30205_TEMP_SCALE * raw_temp_c) + MAX30205_TEMP_BIAS_C + runtime_offset_c;
#else
    est_temp_c = raw_temp_c + MAX30205_TEMP_OFFSET_C + runtime_offset_c;
#endif

    /*
     * If estimated body temperature is above ambient, apply a configurable
     * ambient-delta gain to compensate for heat loss toward air/PCB.
     */

    if (isfinite(ambient_temp_c)) {
        const float delta_c = est_temp_c - ambient_temp_c;
        if (delta_c > 0.0f) {
            est_temp_c += (MAX30205_AMBIENT_DELTA_GAIN * delta_c);
        }
    }

    return fminf(fmaxf(est_temp_c, 20.0f), 45.0f);
}

static max30102_spo2_sample_rate_t app_logic_map_sample_rate(uint32_t sample_rate_hz)
{
    if (sample_rate_hz <= 75U) {
        return MAX30102_SPO2_SR_50;
    }
    if (sample_rate_hz <= 150U) {
        return MAX30102_SPO2_SR_100;
    }
    if (sample_rate_hz <= 300U) {
        return MAX30102_SPO2_SR_200;
    }
    if (sample_rate_hz <= 600U) {
        return MAX30102_SPO2_SR_400;
    }
    if (sample_rate_hz <= 900U) {
        return MAX30102_SPO2_SR_800;
    }
    if (sample_rate_hz <= 1300U) {
        return MAX30102_SPO2_SR_1000;
    }
    if (sample_rate_hz <= 2400U) {
        return MAX30102_SPO2_SR_1600;
    }
    return MAX30102_SPO2_SR_3200;
}

static bme280_filter_t app_logic_map_bme280_filter(uint32_t coeff)
{
    if (coeff == 0U) {
        return BME280_FILTER_OFF;
    }
    if (coeff <= 2U) {
        return BME280_FILTER_2;
    }
    if (coeff <= 4U) {
        return BME280_FILTER_4;
    }
    if (coeff <= 8U) {
        return BME280_FILTER_8;
    }
    return BME280_FILTER_16;
}

static void app_logic_mark_sensor_offline(app_sensor_slot_t *slot, const char *name)
{
    slot->ready = false;
    slot->next_retry_at_ms = (uint32_t)esp_log_timestamp() + CONFIG_APP_SENSOR_INIT_RETRY_MS;
    ESP_LOGW(TAG, "%s unavailable, retry in %d ms", name, CONFIG_APP_SENSOR_INIT_RETRY_MS);
}

/*
 * Evaluate MAX30205 contact quality using body-vs-ambient delta.
 *
 * Rationale:
 * - A true skin contact should usually sit above ambient temperature.
 * - If delta is too small, body estimate is marked weak/unknown.
 */
static void app_logic_update_contact_quality(app_sensor_readings_t *r)
{
    r->body_ambient_delta_valid = false;
    r->contact_quality_known = false;
    r->contact_quality_valid = false;

    if (!r->raw_body_temp_valid || !r->env_temp_valid) {
        return;
    }

    const float contact_temp_c = r->filtered_body_temp_valid ? r->filtered_body_temp_c : r->raw_body_temp_c;

    r->body_ambient_delta_c = contact_temp_c - r->env_temp_c;
    r->body_ambient_delta_valid = true;
    r->contact_quality_known = true;
    const bool meets_min_contact_temp = (MAX30205_CONTACT_BODY_MIN_C <= 0.0f) || (contact_temp_c >= MAX30205_CONTACT_BODY_MIN_C);
    r->contact_quality_valid = (r->body_ambient_delta_c >= MAX30205_CONTACT_DELTA_MIN_C) && meets_min_contact_temp;
}

/*
 * Suppress estimated body output when contact quality is weak.
 *
 * We keep raw/filtered channels for debugging, but prevent downstream logic
 * from treating poor-contact estimates as valid body temperature.
 */
static void app_logic_apply_body_contact_gate(app_sensor_readings_t *r)
{
    if (r == NULL) {
        return;
    }

    if (!r->contact_quality_known || r->contact_quality_valid) {
        return;
    }

    /* Keep raw channels for diagnostics, suppress estimated body channel on weak contact. */
    r->est_body_temp_valid = false;
    r->est_body_temp_c = NAN;
    r->body_temp_valid = false;
    r->body_temp_c = NAN;
}

static void app_logic_log_i2c_scan_on_startup(void)
{
    i2c_manager_scan_result_t scan_result;
    const esp_err_t err = i2c_manager_scan(&scan_result, 25U);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Startup I2C scan complete: %u device(s) found", (unsigned)scan_result.count);
        return;
    }

    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Startup I2C scan found no devices");
        return;
    }

    ESP_LOGW(TAG, "Startup I2C scan failed: %s", esp_err_to_name(err));
}

/*
 * Lazy init for MAX30205 with retry backoff.
 *
 * Sensor is retried periodically when unavailable to avoid log flooding.
 */
static void app_logic_try_init_max30205(app_context_t *ctx)
{
    const uint32_t now = (uint32_t)esp_log_timestamp();
    if (ctx->max30205_slot.ready || (now < ctx->max30205_slot.next_retry_at_ms)) {
        return;
    }

    const max30205_device_config_t cfg = {
        .address = (uint8_t)CONFIG_APP_MAX30205_ADDR,
        .timeout_ms = 100U,
        .config_reg = 0,
        .low_threshold_c = 35.0f,
        .high_threshold_c = 38.0f,
        .apply_thresholds = true,
    };

    const esp_err_t err = max30205_init(&ctx->max30205, &cfg);
    if (err == ESP_OK) {
        app_logic_max30205_filter_reset(ctx);
        ctx->max30205_slot.ready = true;
        ESP_LOGI(TAG, "MAX30205 ready");
    } else {
        app_logic_mark_sensor_offline(&ctx->max30205_slot, "MAX30205");
    }
}

/*
 * Lazy init for MAX30102 with configured optical/ADC parameters.
 *
 * On success we also reset algorithm state so HR/SpO2 starts from clean history.
 */
static void app_logic_try_init_max30102(app_context_t *ctx)
{
    const uint32_t now = (uint32_t)esp_log_timestamp();
    if (ctx->max30102_slot.ready || (now < ctx->max30102_slot.next_retry_at_ms)) {
        return;
    }

    const max30102_spo2_sample_rate_t sample_rate = app_logic_map_sample_rate(MAX30102_SAMPLE_RATE);
    const max30102_config_t cfg = {
        .address = (uint8_t)CONFIG_APP_MAX30102_ADDR,
        .timeout_ms = 100U,
        .sample_average = (max30102_sample_average_t)MAX30102_SAMPLE_AVERAGING,
        .fifo_rollover_enable = true,
        .fifo_almost_full = 0x0F,
        .mode = MAX30102_MODE_SPO2,
        .adc_range = MAX30102_ADC_RANGE_16384,
        .sample_rate = sample_rate,
        .pulse_width = MAX30102_PW_411US,
        .led_red_pa = (uint8_t)MAX30102_LED_RED_CURRENT,
        .led_ir_pa = (uint8_t)MAX30102_LED_IR_CURRENT,
        .led_pilot_pa = (uint8_t)CONFIG_APP_MAX30102_LED_PILOT_PA,
        .interrupt_enable_1 = 0x00,
        .interrupt_enable_2 = 0x00,
    };

    const esp_err_t err = max30102_init(&ctx->max30102, &cfg);
    if (err == ESP_OK) {
        ctx->max30102_slot.ready = true;
        ctx->max30102_read_error_streak = 0U;
        max30102_algo_init(&ctx->ppg_algo, max30102_get_sample_rate_hz(&ctx->max30102));
        memset(&ctx->ppg_processed, 0, sizeof(ctx->ppg_processed));
        ESP_LOGI(TAG,
                 "MAX30102 ready (sr=%uHz avg=%u red_led=0x%02X ir_led=0x%02X, ir_finger=%.0f weak=%.0f)",
                 (unsigned)MAX30102_SAMPLE_RATE,
                 (unsigned)MAX30102_SAMPLE_AVERAGING,
                 (unsigned)MAX30102_LED_RED_CURRENT,
                 (unsigned)MAX30102_LED_IR_CURRENT,
                 MAX30102_IR_FINGER_THRESHOLD,
                 MAX30102_IR_WEAK_CONTACT_THRESHOLD);
    } else {
        app_logic_mark_sensor_offline(&ctx->max30102_slot, "MAX30102");
    }
}

static void app_logic_try_init_bme280(app_context_t *ctx)
{
    const uint32_t now = (uint32_t)esp_log_timestamp();
    if (ctx->bme280_slot.ready || (now < ctx->bme280_slot.next_retry_at_ms)) {
        return;
    }

    const bme280_config_t cfg = {
        .address = (uint8_t)CONFIG_APP_BME280_ADDR,
        .timeout_ms = 100U,
        .osrs_t = (bme280_oversampling_t)CONFIG_APP_BME280_TEMP_OS,
        .osrs_p = (bme280_oversampling_t)CONFIG_APP_BME280_PRESS_OS,
        .osrs_h = (bme280_oversampling_t)CONFIG_APP_BME280_HUM_OS,
#if CONFIG_APP_BME280_USE_FORCED_MODE
        .mode = BME280_MODE_FORCED,
#else
        .mode = BME280_MODE_NORMAL,
#endif
        .standby_time = BME280_STANDBY_500_MS,
        .filter = app_logic_map_bme280_filter(CONFIG_APP_BME280_FILTER_COEFF),
    };

    const esp_err_t err = bme280_init(&ctx->bme280, &cfg);
    if (err == ESP_OK) {
        ctx->bme280_slot.ready = true;
        ESP_LOGI(TAG, "BME280 ready");
    } else {
        app_logic_mark_sensor_offline(&ctx->bme280_slot, "BME280");
    }
}

static void app_logic_try_init_sensors(app_context_t *ctx)
{
    app_logic_try_init_max30205(ctx);
    app_logic_try_init_max30102(ctx);
    app_logic_try_init_bme280(ctx);
}

/*
 * Read MAX30205 and populate raw/filtered/estimated body channels.
 *
 * Data path:
 * raw sensor -> moving average -> calibration model -> body channel.
 */
static void app_logic_read_max30205(app_context_t *ctx, app_sensor_readings_t *out)
{
    if (!ctx->max30205_slot.ready) {
        return;
    }

    float raw_temp_c = NAN;
    const esp_err_t err = max30205_read_temperature_c(&ctx->max30205, &raw_temp_c);
    if (err == ESP_OK) {
        out->raw_body_temp_c = raw_temp_c;
        out->raw_body_temp_valid = true;

        out->filtered_body_temp_c = app_logic_max30205_filter_update(ctx, raw_temp_c);
        out->filtered_body_temp_valid = true;

        out->est_body_temp_c = max30205_apply_calibration(ctx, out->filtered_body_temp_c, out->env_temp_c);
        out->est_body_temp_valid = true;

        /* Keep existing body-temp channel as estimated wearable body temperature. */
        out->body_temp_c = out->est_body_temp_c;
        out->body_temp_valid = true;
        return;
    }

    ESP_LOGW(TAG, "MAX30205 read failed: %s", esp_err_to_name(err));
    (void)max30205_deinit(&ctx->max30205);
    app_logic_max30205_filter_reset(ctx);
    app_logic_mark_sensor_offline(&ctx->max30205_slot, "MAX30205");
}

/*
 * Drain MAX30102 FIFO and update HR/SpO2 algorithm state.
 *
 * Notes for beginners:
 * - We may read multiple FIFO samples per scheduler cycle to catch up.
 * - Algorithm consumes each sample sequentially to preserve timing history.
 * - Read errors are counted; repeated failures trigger sensor re-init path.
 */
static void app_logic_read_max30102(app_context_t *ctx, app_sensor_readings_t *out)
{
    if (!ctx->max30102_slot.ready) {
        return;
    }

    bool received_any = false;
    max30102_sample_t sample;

    /*
     * Read up to a fixed budget of samples to avoid starving other tasks.
     * If FIFO empties early, max30102_fifo_read_sample returns ESP_ERR_NOT_FOUND.
     */
    for (int i = 0; i < MAX30102_FIFO_READ_BUDGET_PER_CYCLE; ++i) {
        const esp_err_t err = max30102_fifo_read_sample(&ctx->max30102, &sample);
        if (err == ESP_ERR_NOT_FOUND) {
            break;
        }
        if (err != ESP_OK) {
            ctx->max30102_read_error_streak++;
            ESP_LOGW(TAG,
                     "MAX30102 FIFO read failed (%u/%u): %s",
                     (unsigned)ctx->max30102_read_error_streak,
                     (unsigned)MAX30102_MAX_READ_ERRORS_BEFORE_RESET,
                     esp_err_to_name(err));
            if (ctx->max30102_read_error_streak >= MAX30102_MAX_READ_ERRORS_BEFORE_RESET) {
                (void)max30102_deinit(&ctx->max30102);
                app_logic_mark_sensor_offline(&ctx->max30102_slot, "MAX30102");
            }
            break;
        }

        ctx->max30102_read_error_streak = 0U;
        received_any = true;
        out->red_raw = sample.red;
        out->ir_raw = sample.ir;
        max30102_algo_update(&ctx->ppg_algo, &sample, &ctx->ppg_processed);
    }

    if (received_any) {
        out->ppg_raw_valid = true;
    }

    out->red_filtered = ctx->ppg_processed.red_filtered;
    out->ir_filtered = ctx->ppg_processed.ir_filtered;
    out->red_dc = ctx->ppg_processed.red_dc;
    out->ir_dc = ctx->ppg_processed.ir_dc;
    out->red_ac_rms = ctx->ppg_processed.red_ac_rms;
    out->ir_ac_rms = ctx->ppg_processed.ir_ac_rms;
    out->ppg_filtered_valid = (ctx->ppg_algo.sample_count > 0U);
    out->ppg_quality_valid = (ctx->ppg_algo.sample_count > 0U);
    out->ppg_contact_quality = ctx->ppg_processed.contact_quality;
    out->ppg_signal_saturated = ctx->ppg_processed.signal_saturated;
    out->heart_rate_confidence = ctx->ppg_processed.heart_rate_confidence;
    out->spo2_confidence = ctx->ppg_processed.spo2_confidence;
    out->hr_reason = ctx->ppg_processed.hr_reason;
    out->spo2_reason = ctx->ppg_processed.spo2_reason;

    if (ctx->ppg_processed.heart_rate_valid) {
        out->heart_rate_valid = true;
        out->heart_rate_bpm = ctx->ppg_processed.heart_rate_bpm;
    }
    if (ctx->ppg_processed.spo2_valid) {
        out->spo2_valid = true;
        out->spo2_percent = ctx->ppg_processed.spo2_percent;
    }
}

static void app_logic_read_bme280(app_context_t *ctx, app_sensor_readings_t *out)
{
    if (!ctx->bme280_slot.ready) {
        return;
    }

    bme280_measurements_t m;
    const esp_err_t err = bme280_read_measurements(&ctx->bme280, &m);
    if (err == ESP_OK) {
        out->env_temp_c = m.temperature_c;
        out->humidity_percent = m.humidity_percent;
        out->pressure_hpa = m.pressure_hpa;
        out->env_temp_valid = true;
        out->humidity_valid = true;
        out->pressure_valid = true;
        return;
    }

    ESP_LOGW(TAG, "BME280 read failed: %s", esp_err_to_name(err));
    (void)bme280_deinit(&ctx->bme280);
    app_logic_mark_sensor_offline(&ctx->bme280_slot, "BME280");
}

/*
 * Build one complete application snapshot:
 * - initialize all fields to safe NA defaults
 * - read sensors in dependency order (ambient first for body delta/calibration)
 * - apply contact gating
 * - compute heat-stroke risk and alert outputs
 */
static void app_logic_read_all(app_context_t *ctx, app_sensor_readings_t *out)
{
    memset(out, 0, sizeof(*out));
    out->body_temp_c = NAN;
    out->raw_body_temp_c = NAN;
    out->filtered_body_temp_c = NAN;
    out->est_body_temp_c = NAN;
    out->body_ambient_delta_c = NAN;
    out->red_filtered = NAN;
    out->ir_filtered = NAN;
    out->red_dc = NAN;
    out->ir_dc = NAN;
    out->red_ac_rms = NAN;
    out->ir_ac_rms = NAN;
    out->heart_rate_bpm = NAN;
    out->heart_rate_confidence = 0.0f;
    out->hr_reason = MAX30102_REASON_INSUFFICIENT_SAMPLES;
    out->spo2_percent = NAN;
    out->spo2_confidence = 0.0f;
    out->spo2_reason = MAX30102_REASON_INSUFFICIENT_SAMPLES;
    out->ppg_contact_quality = MAX30102_CONTACT_NO_FINGER;
    out->heat_index_c = NAN;
    out->total_risk = 0;
    out->raw_risk_category = APP_RISK_CATEGORY_NORMAL;
    out->stable_risk_category = APP_RISK_CATEGORY_NORMAL;
    out->buzzer_mode = APP_BUZZER_MODE_OFF;
    out->env_temp_c = NAN;
    out->humidity_percent = NAN;
    out->pressure_hpa = NAN;

    app_logic_read_bme280(ctx, out);
    app_logic_read_max30205(ctx, out);
    app_logic_read_max30102(ctx, out);
    app_logic_update_contact_quality(out);
    app_logic_apply_body_contact_gate(out);
    app_logic_compute_heatstroke_risk(ctx, out);
}

static const char *app_logic_format_float(char *buf, size_t len, bool valid, float value, const char *format)
{
    if (!valid || isnan(value)) {
        (void)snprintf(buf, len, "NA");
    } else {
        (void)snprintf(buf, len, format, value);
    }
    return buf;
}

static const char *app_logic_format_u32(char *buf, size_t len, bool valid, uint32_t value)
{
    if (!valid) {
        (void)snprintf(buf, len, "NA");
    } else {
        (void)snprintf(buf, len, "%" PRIu32, value);
    }
    return buf;
}

static const char *app_logic_contact_quality_string(const app_sensor_readings_t *r)
{
    if (!r->contact_quality_known) {
        return "NA";
    }
    return r->contact_quality_valid ? "valid" : "weak";
}

static const char *app_logic_max30205_cal_mode_name(void)
{
#if MAX30205_CAL_MODE == MAX30205_CAL_MODE_LINEAR
    return "linear";
#else
    return "offset";
#endif
}

/*
 * Consolidated one-line diagnostics log.
 *
 * This line intentionally includes both raw and processed channels so you can
 * debug sensor quality, contact status, and risk decisions together.
 */
static void app_logic_log_readings(const app_sensor_readings_t *r)
{
    char body_temp[16];
    char raw_body[16];
    char filt_body[16];
    char est_body[16];
    char hr[16];
    char spo2[16];
    char red[16];
    char ir[16];
    char red_f[16];
    char ir_f[16];
    char red_dc[16];
    char ir_dc[16];
    char red_ac[16];
    char ir_ac[16];
    char hr_conf[16];
    char spo2_conf[16];
    char env_temp[16];
    char hum[16];
    char heat_index[16];
    char delta[16];
    char pressure[20];

    ESP_LOGI(TAG,
             "BodyTemp=%s C, RawBody=%s C, FiltBody=%s C, EstBody=%s C, EnvTemp=%s C, Delta=%s C, Contact=%s, "
             "HR=%s bpm, SpO2=%s %%, RED=%s, IR=%s, REDf=%s, IRf=%s, REDdc=%s, IRdc=%s, REDac=%s, IRac=%s, "
             "PPGContact=%s, Sat=%s, HRconf=%s, SpO2conf=%s, HRna=%s, SpO2na=%s, Hum=%s %%, HeatIndex=%s C, "
             "Risk[HR=%d%s Body=%d%s HI=%d%s Total=%d Raw=%s Stable=%s Debounce=%u/%u Buzzer=%s], Pressure=%s hPa",
             app_logic_format_float(body_temp, sizeof(body_temp), r->body_temp_valid, r->body_temp_c, "%.2f"),
             app_logic_format_float(raw_body, sizeof(raw_body), r->raw_body_temp_valid, r->raw_body_temp_c, "%.2f"),
             app_logic_format_float(
                 filt_body, sizeof(filt_body), r->filtered_body_temp_valid, r->filtered_body_temp_c, "%.2f"),
             app_logic_format_float(est_body, sizeof(est_body), r->est_body_temp_valid, r->est_body_temp_c, "%.2f"),
             app_logic_format_float(env_temp, sizeof(env_temp), r->env_temp_valid, r->env_temp_c, "%.2f"),
             app_logic_format_float(
                 delta, sizeof(delta), r->body_ambient_delta_valid, r->body_ambient_delta_c, "%.2f"),
             app_logic_contact_quality_string(r),
             app_logic_format_float(hr, sizeof(hr), r->heart_rate_valid, r->heart_rate_bpm, "%.1f"),
             app_logic_format_float(spo2, sizeof(spo2), r->spo2_valid, r->spo2_percent, "%.1f"),
             app_logic_format_u32(red, sizeof(red), r->ppg_raw_valid, r->red_raw),
             app_logic_format_u32(ir, sizeof(ir), r->ppg_raw_valid, r->ir_raw),
             app_logic_format_float(red_f, sizeof(red_f), r->ppg_filtered_valid, r->red_filtered, "%.0f"),
             app_logic_format_float(ir_f, sizeof(ir_f), r->ppg_filtered_valid, r->ir_filtered, "%.0f"),
             app_logic_format_float(red_dc, sizeof(red_dc), r->ppg_quality_valid, r->red_dc, "%.0f"),
             app_logic_format_float(ir_dc, sizeof(ir_dc), r->ppg_quality_valid, r->ir_dc, "%.0f"),
             app_logic_format_float(red_ac, sizeof(red_ac), r->ppg_quality_valid, r->red_ac_rms, "%.1f"),
             app_logic_format_float(ir_ac, sizeof(ir_ac), r->ppg_quality_valid, r->ir_ac_rms, "%.1f"),
             r->ppg_quality_valid ? max30102_contact_quality_to_string(r->ppg_contact_quality) : "NA",
             r->ppg_quality_valid ? (r->ppg_signal_saturated ? "yes" : "no") : "NA",
             app_logic_format_float(hr_conf, sizeof(hr_conf), r->ppg_quality_valid, r->heart_rate_confidence, "%.2f"),
             app_logic_format_float(
                 spo2_conf, sizeof(spo2_conf), r->ppg_quality_valid, r->spo2_confidence, "%.2f"),
             max30102_reason_to_string(r->hr_reason),
             max30102_reason_to_string(r->spo2_reason),
             app_logic_format_float(hum, sizeof(hum), r->humidity_valid, r->humidity_percent, "%.2f"),
             app_logic_format_float(heat_index, sizeof(heat_index), r->heat_index_valid, r->heat_index_c, "%.2f"),
             r->heart_rate_risk,
             r->heart_rate_risk_applied ? "" : "(HR_INVALID)",
             r->body_temp_risk,
             r->body_temp_risk_applied ? "" : "(BT_INVALID)",
             r->heat_index_risk,
             r->heat_index_risk_applied ? "" : "(HI_INVALID)",
             r->total_risk,
             app_logic_risk_category_to_string(r->raw_risk_category),
             app_logic_risk_category_to_string(r->stable_risk_category),
             (unsigned)s_app_ctx.pending_risk_category_count,
             (unsigned)CONFIG_APP_RISK_DEBOUNCE_CYCLES,
             app_logic_buzzer_mode_to_string(r->buzzer_mode),
             app_logic_format_float(pressure, sizeof(pressure), r->pressure_valid, r->pressure_hpa, "%.2f"));
}

static void app_sensor_task(void *arg)
{
    app_context_t *ctx = (app_context_t *)arg;
    app_sensor_readings_t readings;
    uint32_t last_loop_log_ms = 0U;

    while (true) {
        app_logic_try_init_sensors(ctx);
        app_logic_read_all(ctx, &readings);
        app_logic_publish_ble_telemetry(ctx, &readings);
#if APP_ENABLE_SENSOR_LOOP_LOG
        const uint32_t now_ms = (uint32_t)esp_log_timestamp();
        if ((now_ms - last_loop_log_ms) >= APP_SENSOR_LOOP_LOG_INTERVAL_MS) {
            last_loop_log_ms = now_ms;
            app_logic_log_readings(&readings);
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(CONFIG_APP_SENSOR_TASK_PERIOD_MS));
    }
}

esp_err_t app_logic_start(void)
{
    if (s_app_ctx.started) {
        return ESP_OK;
    }

    const uint32_t i2c_hz = (uint32_t)CONFIG_APP_I2C_FREQ_HZ;

    const i2c_manager_config_t i2c_cfg = {
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_APP_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_APP_I2C_SCL_GPIO,
        .clk_speed_hz = i2c_hz,
        .glitch_ignore_cnt = 7,
        .enable_internal_pullup = true,
    };

    esp_err_t err = i2c_manager_init(&i2c_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return err;
    }

    app_logic_log_i2c_scan_on_startup();

    memset(&s_app_ctx, 0, sizeof(s_app_ctx));
    app_logic_max30205_filter_reset(&s_app_ctx);
    s_app_ctx.pending_risk_category = APP_RISK_CATEGORY_NORMAL;
    s_app_ctx.stable_risk_category = APP_RISK_CATEGORY_NORMAL;
    s_app_ctx.buzzer_mode = APP_BUZZER_MODE_OFF;
    s_app_ctx.ble_runtime_temp_offset_c = 0.0f;
    s_app_ctx.ble_buzzer_enabled = true;
    s_app_ctx.sample_counter = 0U;
    s_app_ctx.started = true;

    err = app_logic_output_init(&s_app_ctx);
    if (err != ESP_OK) {
        s_app_ctx.started = false;
        (void)i2c_manager_deinit();
        return err;
    }

#if CONFIG_APP_BLE_ENABLE
    const ble_manager_config_t ble_cfg = {
        .device_name = CONFIG_APP_BLE_DEVICE_NAME,
        .notify_interval_ms = (uint16_t)CONFIG_APP_BLE_NOTIFY_INTERVAL_MS,
        .buzzer_enabled = 1U,
        .body_temp_offset_c_x100 = 0,
        .debug_log_level = 3U,
        .settings_cb = app_logic_ble_settings_callback,
        .command_cb = app_logic_ble_command_callback,
        .user_ctx = &s_app_ctx,
    };
    err = ble_manager_init(&ble_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed: %s", esp_err_to_name(err));
        s_app_ctx.started = false;
#if (CONFIG_APP_BUZZER_GPIO >= 0) || (CONFIG_APP_STATUS_LED_GPIO >= 0)
        if (s_app_ctx.output_timer_started && (s_app_ctx.output_timer != NULL)) {
            (void)esp_timer_stop(s_app_ctx.output_timer);
            (void)esp_timer_delete(s_app_ctx.output_timer);
            s_app_ctx.output_timer = NULL;
            s_app_ctx.output_timer_started = false;
        }
#endif
        (void)i2c_manager_deinit();
        return err;
    }
#endif

    ESP_LOGI(TAG,
             "MAX30205 calibration: mode=%s offset=%.3f scale=%.3f bias=%.3f runtime_ble_offset=%.2f "
             "contact_delta_min=%.2fC contact_temp_min=%.2fC ambient_gain=%.2f ma_window=%u",
             app_logic_max30205_cal_mode_name(),
             MAX30205_TEMP_OFFSET_C,
             MAX30205_TEMP_SCALE,
             MAX30205_TEMP_BIAS_C,
             s_app_ctx.ble_runtime_temp_offset_c,
             MAX30205_CONTACT_DELTA_MIN_C,
             MAX30205_CONTACT_BODY_MIN_C,
             MAX30205_AMBIENT_DELTA_GAIN,
             (unsigned)MAX30205_FILTER_WINDOW_SAMPLES);
    ESP_LOGI(TAG,
             "MAX30102 tuning: sr=%uHz avg=%u led_red=0x%02X led_ir=0x%02X ir_finger=%.0f ir_weak=%.0f "
             "buf=%u beats_min=%u conf_min=%.2f ac_min=%.1f ac_max=%.1f",
             (unsigned)MAX30102_SAMPLE_RATE,
             (unsigned)MAX30102_SAMPLE_AVERAGING,
             (unsigned)MAX30102_LED_RED_CURRENT,
             (unsigned)MAX30102_LED_IR_CURRENT,
             MAX30102_IR_FINGER_THRESHOLD,
             MAX30102_IR_WEAK_CONTACT_THRESHOLD,
             (unsigned)MAX30102_BUFFER_SIZE,
             (unsigned)MAX30102_MIN_VALID_BEATS,
             MAX30102_MIN_CONFIDENCE,
             MAX30102_SIGNAL_AC_MIN,
             MAX30102_SIGNAL_AC_MAX);
    ESP_LOGI(TAG,
             "Heat-stroke risk model: debounce=%u cycles, buzzer_gpio=%d led_gpio=%d",
             (unsigned)CONFIG_APP_RISK_DEBOUNCE_CYCLES,
             CONFIG_APP_BUZZER_GPIO,
             CONFIG_APP_STATUS_LED_GPIO);
#if CONFIG_APP_BLE_ENABLE
    ESP_LOGI(TAG,
             "BLE stream enabled: name=%s notify_interval=%u ms service=%s",
             CONFIG_APP_BLE_DEVICE_NAME,
             (unsigned)CONFIG_APP_BLE_NOTIFY_INTERVAL_MS,
             "12345678-1234-5678-1234-56789abc0000");
#else
    ESP_LOGW(TAG, "BLE stream disabled by CONFIG_APP_BLE_ENABLE");
#endif

    const BaseType_t task_ok = xTaskCreate(app_sensor_task,
                                           "sensor_task",
                                           CONFIG_APP_SENSOR_TASK_STACK_SIZE,
                                           &s_app_ctx,
                                           CONFIG_APP_SENSOR_TASK_PRIORITY,
                                           &s_task_handle);
    if (task_ok != pdPASS) {
        s_app_ctx.started = false;
#if (CONFIG_APP_BUZZER_GPIO >= 0) || (CONFIG_APP_STATUS_LED_GPIO >= 0)
        if (s_app_ctx.output_timer_started && (s_app_ctx.output_timer != NULL)) {
            (void)esp_timer_stop(s_app_ctx.output_timer);
            (void)esp_timer_delete(s_app_ctx.output_timer);
            s_app_ctx.output_timer = NULL;
            s_app_ctx.output_timer_started = false;
        }
#endif
        (void)i2c_manager_deinit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor task started (%d ms period)", CONFIG_APP_SENSOR_TASK_PERIOD_MS);
    return ESP_OK;
}
