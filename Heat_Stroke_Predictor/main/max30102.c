#include "max30102.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * MAX30102 register map (subset used by this driver).
 *
 * Why use named constants:
 * - Register writes are easy to misread when using raw hex literals.
 * - Names document intent and reduce mistakes when editing configs later.
 */
#define MAX30102_REG_INTR_STATUS_1 0x00U
#define MAX30102_REG_INTR_STATUS_2 0x01U
#define MAX30102_REG_INTR_ENABLE_1 0x02U
#define MAX30102_REG_INTR_ENABLE_2 0x03U
/*
 * FIFO core registers (0x04..0x07):
 * - 0x04 FIFO_WR_PTR: write pointer advanced by sensor sampling engine.
 * - 0x05 OVF_COUNTER: counts FIFO overruns when host reads too slowly.
 * - 0x06 FIFO_RD_PTR: read pointer advanced as host drains FIFO data.
 * - 0x07 FIFO_DATA: streaming data port used for burst payload reads.
 */
#define MAX30102_REG_FIFO_WR_PTR 0x04U /* FIFO Write Pointer: sensor-side next write location */
#define MAX30102_REG_OVF_COUNTER 0x05U /* FIFO Overflow Counter: increments when unread data is overrun */
#define MAX30102_REG_FIFO_RD_PTR 0x06U /* FIFO Read Pointer: host-side next unread sample location */
#define MAX30102_REG_FIFO_DATA 0x07U /* FIFO Data Port: burst reads pop sequential FIFO bytes */
#define MAX30102_REG_FIFO_CONFIG 0x08U
#define MAX30102_REG_MODE_CONFIG 0x09U
#define MAX30102_REG_SPO2_CONFIG 0x0AU
#define MAX30102_REG_LED1_PA 0x0CU
#define MAX30102_REG_LED2_PA 0x0DU
#define MAX30102_REG_PILOT_PA 0x10U
#define MAX30102_REG_MULTI_LED_CTRL1 0x11U
#define MAX30102_REG_MULTI_LED_CTRL2 0x12U
#define MAX30102_REG_TEMP_INT 0x1FU
#define MAX30102_REG_TEMP_FRAC 0x20U
#define MAX30102_REG_TEMP_CONFIG 0x21U
#define MAX30102_REG_PART_ID 0xFFU

#define MAX30102_PART_ID 0x15U

/*
 * Control bit definitions.
 *
 * Writing these bits controls state-machine operations inside the sensor.
 * Keeping them as named masks makes bitwise code much clearer for beginners.
 */
#define MAX30102_MODE_RESET_BIT (1U << 6)
#define MAX30102_TEMP_START_CONVERSION_BIT (1U << 0)
#define MAX30102_TEMP_RDY_INT_BIT (1U << 1)

/*
 * Driver-level tuning constants.
 *
 * These are not hardware registers. They are software gates used by:
 * - contact classification
 * - beat/HR quality checks
 * - SpO2 confidence and hold behavior
 */
#define MAX30102_TIMEOUT_DEFAULT_MS 100U
#define MAX30102_ADDRESS_DEFAULT 0x57U
#define MAX30102_SIGNAL_SATURATION_LEVEL 250000.0f
#define MAX30102_MIN_ESTIMATION_SECONDS 1.0f
#define MAX30102_MAX_HR_BPM 210.0f
#define MAX30102_MIN_HR_BPM 45.0f
#define MAX30102_PEAK_THRESHOLD_MIN 6.0f
#define MAX30102_PEAK_PROMINENCE_MIN 6.0f
#define MAX30102_PEAK_THRESHOLD_RMS_SCALE 0.05f
#define MAX30102_PEAK_THRESHOLD_DERIV_SCALE 2.20f
#define MAX30102_DET_BASELINE_ALPHA 0.02f
#define MAX30102_PEAK_BOOTSTRAP_THRESHOLD_SCALE 0.70f
#define MAX30102_PEAK_PROMINENCE_RATIO_MIN 0.06f
#define MAX30102_PEAK_PROMINENCE_ABS_MIN 2.5f
#define MAX30102_PEAK_SLOPE_RATIO_MIN 0.10f
#define MAX30102_PEAK_SLOPE_ABS_MIN 2.0f
#define MAX30102_MOTION_NOISE_RATIO_MAX 3.5f
#define MAX30102_VALUE_HOLD_SECONDS 2.0f
#define MAX30102_HR_STALE_SECONDS 2.6f
#define MAX30102_DEBUG_LOG_INTERVAL_MS 1000U
#define MAX30102_VALID_ENTER_STABLE_MS 500U
#define MAX30102_VALID_ENTER_IR_DC_THRESHOLD MAX30102_IR_WEAK_CONTACT_THRESHOLD
#define MAX30102_VALID_EXIT_IR_DC_THRESHOLD (MAX30102_IR_WEAK_CONTACT_THRESHOLD * 0.70f)
#define MAX30102_VALID_EXIT_IR_RAW_THRESHOLD (MAX30102_IR_FINGER_THRESHOLD * 0.80f)
#define MAX30102_VALID_ENTER_AC_MIN MAX30102_SIGNAL_AC_MIN
#define MAX30102_VALID_EXIT_AC_MIN (MAX30102_SIGNAL_AC_MIN * 0.55f)
#define MAX30102_VALID_EXIT_BAD_SAMPLES 6U
#define MAX30102_VALID_EXIT_MOTION_RATIO (MAX30102_MOTION_NOISE_RATIO_MAX * 1.35f)

static const char *TAG = "max30102";
#if MAX30102_ENABLE_EVENT_LOG
#define MAX30102_EVT_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define MAX30102_EVT_LOGI(...) ((void)0)
#endif
#if MAX30102_SAMPLE_LOG_TO_FILE
/* Log file open failures are reported once, then logger falls back to ESP_LOGI. */
static bool s_sample_log_file_open_warned = false;
#endif
/* Global debug context used by FIFO-side raw logging. */
static uint32_t s_sample_log_counter = 0U;
static max30102_contact_quality_t s_sample_log_contact = MAX30102_CONTACT_NO_FINGER;

/*
 * Convert contact enum to stable log text.
 *
 * Why this helper exists:
 * - log formatting should not duplicate switch statements everywhere.
 * - keeps log wording consistent across the whole driver.
 */
static const char *max30102_contact_quality_to_cstr(max30102_contact_quality_t quality)
{
    switch (quality) {
    case MAX30102_CONTACT_NO_FINGER:
        return "no_contact";
    case MAX30102_CONTACT_WEAK:
        return "weak_contact";
    case MAX30102_CONTACT_VALID:
        return "valid_contact";
    default:
        return "unknown";
    }
}

static const char *max30102_reason_to_cstr(max30102_reason_t reason)
{
    /*
     * These strings map 1:1 with reason enum values.
     * App logs can show these directly to explain why value is NA.
     */
    switch (reason) {
    case MAX30102_REASON_NONE:
        return "ok";
    case MAX30102_REASON_NO_CONTACT:
        return "no_contact";
    case MAX30102_REASON_WEAK_CONTACT:
        return "weak_contact";
    case MAX30102_REASON_INSUFFICIENT_SAMPLES:
        return "insufficient_samples";
    case MAX30102_REASON_SIGNAL_WEAK:
        return "signal_weak";
    case MAX30102_REASON_SIGNAL_SATURATED:
        return "signal_saturated";
    case MAX30102_REASON_SIGNAL_NOISY:
        return "signal_noisy";
    case MAX30102_REASON_UNSTABLE_BEATS:
        return "unstable_beats";
    case MAX30102_REASON_INVALID_RATIO:
        return "invalid_ratio";
    default:
        return "unknown";
    }
}

/*
 * Raw sample logger for offline debugging of HR/SpO2 lock behavior.
 * Main path is ESP_LOGI (capture monitor output into max30102.log on host).
 * Optional file logging can be enabled if target has writable VFS storage.
 */
static void max30102_log_raw_sample(const max30102_sample_t *sample,
                                    uint32_t sample_index,
                                    max30102_contact_quality_t contact_quality)
{
#if MAX30102_ENABLE_SAMPLE_LOG
    if (sample == NULL) {
        return;
    }

    /*
     * Optional file logging path.
     * This is usually disabled on bare MCU targets unless VFS storage is mounted.
     */
#if MAX30102_SAMPLE_LOG_TO_FILE
    FILE *fp = fopen(MAX30102_SAMPLE_LOG_FILE_PATH, "a");
    if (fp != NULL) {
        (void)fprintf(fp,
                      "%" PRIu32 " ms, idx=%" PRIu32 ", RED=%" PRIu32 ", IR=%" PRIu32 ", contact=%s\n",
                      sample->timestamp_ms,
                      sample_index,
                      sample->red,
                      sample->ir,
                      max30102_contact_quality_to_cstr(contact_quality));
        (void)fclose(fp);
        return;
    }

    if (!s_sample_log_file_open_warned) {
        s_sample_log_file_open_warned = true;
        ESP_LOGW(TAG,
                 "sample file logging unavailable (%s). Falling back to ESP_LOGI output.",
                 MAX30102_SAMPLE_LOG_FILE_PATH);
    }
#endif

    /*
     * Default logging path:
     * - Always available in ESP-IDF.
     * - Host can capture monitor output into a max30102.log file.
     */
    ESP_LOGI(TAG,
             "RAW_SAMPLE %" PRIu32 " ms, idx=%" PRIu32 ", RED=%" PRIu32 ", IR=%" PRIu32 ", contact=%s",
             sample->timestamp_ms,
             sample_index,
             sample->red,
             sample->ir,
             max30102_contact_quality_to_cstr(contact_quality));
#else
    (void)sample;
    (void)sample_index;
    (void)contact_quality;
#endif
}

static bool max30102_time_is_before(uint32_t now_ms, uint32_t target_ms)
{
    /*
     * Wrap-safe comparison for uint32_t time values.
     * Casting the subtraction result to signed handles timestamp overflow cases.
     */
    return ((int32_t)(target_ms - now_ms) > 0);
}

/*
 * Reset algorithm history when contact session changes significantly.
 *
 * Why a reset is needed:
 * - stale filters/beats from previous contact can pollute new readings.
 * - especially important when finger is removed/re-placed.
 *
 * Why we keep seed_red_dc/seed_ir_dc:
 * - avoids a hard jump from old state to zero.
 * - gives filter a realistic starting baseline from current sample.
 */
static void max30102_algo_reset_tracking(max30102_algo_state_t *state,
                                         float seed_red_dc,
                                         float seed_ir_dc,
                                         uint32_t timestamp_ms,
                                         const char *reason)
{
    if (state == NULL) {
        return;
    }

    /* Reset baselines and filtered AC components. */
    state->red_dc = seed_red_dc;
    state->ir_dc = seed_ir_dc;
    state->red_ac_filtered = 0.0f;
    state->ir_ac_filtered = 0.0f;
    state->ir_ac_baseline = 0.0f;
    state->prev_ir_ac = 0.0f;
    state->prev_prev_ir_ac = 0.0f;
    state->sample_count = 0U;
    state->last_peak_index = -1;
    /* Clear rolling AC windows and their accumulated energy sums. */
    memset(state->red_ac_window, 0, sizeof(state->red_ac_window));
    memset(state->ir_ac_window, 0, sizeof(state->ir_ac_window));
    state->ac_window_count = 0U;
    state->ac_window_head = 0U;
    state->red_ac2_sum = 0.0f;
    state->ir_ac2_sum = 0.0f;
    /* Clear beat interval history so previous session beats are forgotten. */
    memset(state->beat_intervals, 0, sizeof(state->beat_intervals));
    state->beat_count = 0U;
    state->beat_head = 0U;
    state->beat_polarity = 0;
    state->peak_candidate_count = 0U;
    state->peak_reject_timing_count = 0U;
    state->peak_reject_shape_count = 0U;
    state->peak_reject_noise_count = 0U;
    state->peak_reject_polarity_count = 0U;
    state->ir_deriv_abs_ema = 0.0f;
    state->last_accepted_peak_index = -1;
    state->last_peak_interval_samples = 0U;
    state->raw_heart_rate_bpm = NAN;
    state->raw_spo2_percent = NAN;
    state->last_ratio = NAN;
    state->last_peak_threshold = 0.0f;
    state->motion_metric = 0.0f;
    state->hr_hold_count = 0U;
    state->spo2_hold_count = 0U;
    state->peak_count = 0U;
    state->last_debug_log_ms = timestamp_ms;
    /* Published outputs are invalid until enough new samples arrive. */
    state->heart_rate_bpm = NAN;
    state->spo2_percent = NAN;
    state->heart_rate_confidence = 0.0f;
    state->spo2_confidence = 0.0f;
    state->hr_reason = MAX30102_REASON_INSUFFICIENT_SAMPLES;
    state->spo2_reason = MAX30102_REASON_INSUFFICIENT_SAMPLES;
    state->heart_rate_valid = false;
    state->spo2_valid = false;
    state->signal_saturated = false;
    state->settling_until_ms = 0U;
    state->contact_transition_ms = timestamp_ms;
    state->valid_candidate_start_ms = 0U;
    state->valid_bad_sample_streak = 0U;

    MAX30102_EVT_LOGI("PPG filter/beat state reset: %s", (reason != NULL) ? reason : "transition");
}

static bool max30102_is_ready(const max30102_t *device)
{
    /* Single readiness gate used by public API functions. */
    return (device != NULL) && device->initialized;
}

static esp_err_t max30102_write_reg(max30102_t *device, uint8_t reg, uint8_t value)
{
    /* Single-byte register write helper around shared I2C manager. */
    return i2c_manager_write_reg_u8(&device->i2c_dev, reg, value, device->timeout_ms);
}

static esp_err_t max30102_read_reg(max30102_t *device, uint8_t reg, uint8_t *value)
{
    /* Single-byte register read helper around shared I2C manager. */
    return i2c_manager_read_reg_u8(&device->i2c_dev, reg, value, device->timeout_ms);
}

static float max30102_clampf(float value, float min_value, float max_value)
{
    /*
     * Basic clamp helper for confidence and range-limited computed values.
     * Keeps downstream code readable by avoiding repeated if-chains.
     */
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void max30102_push_ac_sample(max30102_algo_state_t *state, float red_ac, float ir_ac)
{
    /*
     * Rolling-window buffer update for AC RMS estimation.
     *
     * We track sum of squares incrementally:
     * - add new sample^2
     * - subtract old sample^2 when buffer wraps
     * This avoids expensive full-window recomputation every sample.
     */
    const uint16_t idx = state->ac_window_head;
    const bool full = (state->ac_window_count >= (uint16_t)MAX30102_BUFFER_SIZE);

    if (full) {
        const float old_red = state->red_ac_window[idx];
        const float old_ir = state->ir_ac_window[idx];
        state->red_ac2_sum -= old_red * old_red;
        state->ir_ac2_sum -= old_ir * old_ir;
    } else {
        state->ac_window_count++;
    }

    state->red_ac_window[idx] = red_ac;
    state->ir_ac_window[idx] = ir_ac;
    state->red_ac2_sum += red_ac * red_ac;
    state->ir_ac2_sum += ir_ac * ir_ac;

    state->ac_window_head++;
    if (state->ac_window_head >= (uint16_t)MAX30102_BUFFER_SIZE) {
        state->ac_window_head = 0U;
    }
}

static void max30102_push_interval(max30102_algo_state_t *state, uint16_t interval_samples)
{
    /*
     * Ring-buffer storage for beat intervals (in sample counts).
     * Later converted to BPM using sample_rate_hz.
     */
    const uint8_t idx = state->beat_head;
    state->beat_intervals[idx] = interval_samples;

    if (state->beat_count < (uint8_t)MAX30102_BEAT_HISTORY_SIZE) {
        state->beat_count++;
    }

    state->beat_head++;
    if (state->beat_head >= (uint8_t)MAX30102_BEAT_HISTORY_SIZE) {
        state->beat_head = 0U;
    }
}

/*
 * Return how many samples elapsed since the last accepted beat.
 *
 * This is used to expire stale HR values: if no fresh beat arrives for too
 * long, we stop publishing old BPM as if it were current.
 */
static uint32_t max30102_samples_since_last_accepted_beat(const max30102_algo_state_t *state)
{
    if ((state == NULL) || (state->sample_count == 0U) || (state->last_accepted_peak_index < 0)) {
        return UINT32_MAX;
    }

    const uint32_t current_idx = state->sample_count - 1U;
    if ((uint32_t)state->last_accepted_peak_index > current_idx) {
        return UINT32_MAX;
    }

    return current_idx - (uint32_t)state->last_accepted_peak_index;
}

static bool max30102_estimate_heart_rate(const max30102_algo_state_t *state, float *hr_bpm, float *confidence)
{
    /*
     * HR estimation from beat-interval history.
     *
     * Pipeline:
     * 1) collect non-zero intervals
     * 2) median-based outlier rejection
     * 3) compute mean interval
     * 4) convert interval to BPM
     * 5) compute confidence from variability + number of kept beats
     */
    if ((state->beat_count < (uint8_t)MAX30102_MIN_VALID_BEATS) || (state->sample_rate_hz <= 0.0f)) {
        return false;
    }

    float intervals[MAX30102_BEAT_HISTORY_SIZE];
    uint8_t n = 0U;
    for (uint8_t i = 0; i < state->beat_count; ++i) {
        const float x = (float)state->beat_intervals[i];
        if (x > 0.0f) {
            intervals[n++] = x;
        }
    }
    if (n < (uint8_t)MAX30102_MIN_VALID_BEATS) {
        return false;
    }

    /* Small insertion sort: simple and cheap for short beat-history arrays. */
    for (uint8_t i = 1U; i < n; ++i) {
        const float key = intervals[i];
        uint8_t j = i;
        while ((j > 0U) && (intervals[j - 1U] > key)) {
            intervals[j] = intervals[j - 1U];
            j--;
        }
        intervals[j] = key;
    }

    const float median = intervals[n / 2U];
    if (median <= 0.0f) {
        return false;
    }

    float mean = 0.0f;
    uint8_t kept = 0U;
    /* Keep only intervals near median to reduce motion/outlier impact. */
    for (uint8_t i = 0; i < n; ++i) {
        const float rel = fabsf(intervals[i] - median) / median;
        if (rel <= 0.35f) {
            mean += intervals[i];
            kept++;
        }
    }
    if (kept < (uint8_t)MAX30102_MIN_VALID_BEATS) {
        return false;
    }
    mean /= (float)kept;

    float sq = 0.0f;
    for (uint8_t i = 0; i < state->beat_count; ++i) {
        const float x = (float)state->beat_intervals[i];
        const float rel = fabsf(x - median) / median;
        if (rel <= 0.35f) {
            const float d = x - mean;
            sq += d * d;
        }
    }

    /* CV (stddev/mean) is used as a stability metric. Lower CV -> higher confidence. */
    const float stddev = sqrtf(sq / (float)kept);
    const float cv = (mean > 1e-6f) ? (stddev / mean) : 1.0f;
    const float bpm = 60.0f * (state->sample_rate_hz / mean);
    if ((bpm < MAX30102_MIN_HR_BPM) || (bpm > MAX30102_MAX_HR_BPM)) {
        return false;
    }

    const float consistency_conf = 1.0f - max30102_clampf(cv / 0.35f, 0.0f, 1.0f);
    const float count_conf = max30102_clampf(((float)kept - (float)MAX30102_MIN_VALID_BEATS + 1.0f) / 5.0f, 0.0f, 1.0f);
    const float conf = max30102_clampf((0.65f * consistency_conf) + (0.35f * count_conf), 0.0f, 1.0f);

    *hr_bpm = bpm;
    *confidence = conf;
    return true;
}

static max30102_config_t max30102_default_config(void)
{
    /*
     * Defaults tuned for easier finger capture:
     * - SpO2 mode (RED+IR)
     * - 100 Hz sample rate
     * - 16x averaging
     * - 411 us pulse width, max ADC range
     */
    /*
     * Default values here are intentionally conservative:
     * - strong enough LEDs for common breakout modules,
     * - high ADC range to reduce clipping,
     * - moderate sample rate for manageable CPU + good waveform detail.
     */
    const max30102_config_t cfg = {
        .address = MAX30102_ADDRESS_DEFAULT,
        .timeout_ms = MAX30102_TIMEOUT_DEFAULT_MS,
        .sample_average = MAX30102_SAMPLE_AVERAGING,
        .fifo_rollover_enable = true,
        .fifo_almost_full = 0x0F,
        .mode = MAX30102_MODE_SPO2,
        .adc_range = MAX30102_ADC_RANGE_16384,
        .sample_rate = MAX30102_SPO2_SR_100,
        .pulse_width = MAX30102_PW_411US,
        .led_red_pa = (uint8_t)MAX30102_LED_RED_CURRENT,
        .led_ir_pa = (uint8_t)MAX30102_LED_IR_CURRENT,
        .led_pilot_pa = 0x7F,
        .interrupt_enable_1 = 0x00,
        .interrupt_enable_2 = 0x00,
    };
    return cfg;
}

float max30102_sample_rate_enum_to_hz(max30102_spo2_sample_rate_t sample_rate)
{
    /* Convert register enum to real-world Hz for algorithm timing math. */
    switch (sample_rate) {
    case MAX30102_SPO2_SR_50:
        return 50.0f;
    case MAX30102_SPO2_SR_100:
        return 100.0f;
    case MAX30102_SPO2_SR_200:
        return 200.0f;
    case MAX30102_SPO2_SR_400:
        return 400.0f;
    case MAX30102_SPO2_SR_800:
        return 800.0f;
    case MAX30102_SPO2_SR_1000:
        return 1000.0f;
    case MAX30102_SPO2_SR_1600:
        return 1600.0f;
    case MAX30102_SPO2_SR_3200:
        return 3200.0f;
    default:
        return 100.0f;
    }
}

float max30102_get_sample_rate_hz(const max30102_t *device)
{
    /* Safe accessor used by app layer for algorithm initialization. */
    if (device == NULL) {
        return 0.0f;
    }
    return device->sample_rate_hz;
}

esp_err_t max30102_reset(max30102_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Datasheet reset sequence: set reset bit, then poll until self-cleared. */
    esp_err_t err = max30102_write_reg(device, MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET_BIT);
    if (err != ESP_OK) {
        return err;
    }

    /*
     * Poll reset bit until the internal reset sequence finishes.
     * Timeout protects task from blocking forever on bus/hardware fault.
     */
    for (int i = 0; i < 20; ++i) {
        vTaskDelay(pdMS_TO_TICKS(5));
        uint8_t mode_cfg = 0;
        err = max30102_read_reg(device, MAX30102_REG_MODE_CONFIG, &mode_cfg);
        if (err != ESP_OK) {
            return err;
        }
        if ((mode_cfg & MAX30102_MODE_RESET_BIT) == 0U) {
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t max30102_config_interrupts(max30102_t *device, uint8_t int_enable_1, uint8_t int_enable_2)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Write both interrupt enable bytes exactly as provided by caller. */
    esp_err_t err = max30102_write_reg(device, MAX30102_REG_INTR_ENABLE_1, int_enable_1);
    if (err != ESP_OK) {
        return err;
    }
    return max30102_write_reg(device, MAX30102_REG_INTR_ENABLE_2, int_enable_2);
}

esp_err_t max30102_read_interrupt_status(max30102_t *device, uint8_t *int_status_1, uint8_t *int_status_2)
{
    if ((device == NULL) || (int_status_1 == NULL) || (int_status_2 == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Read both status bytes in one burst transaction.
     * This keeps read operation coherent and reduces bus overhead.
     */
    uint8_t status[2];
    esp_err_t err = i2c_manager_read_reg(&device->i2c_dev, MAX30102_REG_INTR_STATUS_1, status, sizeof(status), device->timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    *int_status_1 = status[0];
    *int_status_2 = status[1];
    return ESP_OK;
}

esp_err_t max30102_fifo_flush(max30102_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Keep FIFO pointers aligned when (re)starting acquisition. */
    esp_err_t err = max30102_write_reg(device, MAX30102_REG_FIFO_WR_PTR, 0);
    if (err != ESP_OK) {
        return err;
    }
    /*
     * Overflow counter is reset too, otherwise stale overflow state can remain.
     */
    err = max30102_write_reg(device, MAX30102_REG_OVF_COUNTER, 0);
    if (err != ESP_OK) {
        return err;
    }
    return max30102_write_reg(device, MAX30102_REG_FIFO_RD_PTR, 0);
}

esp_err_t max30102_init(max30102_t *device, const max30102_config_t *config)
{
    /*
     * Initialization flow:
     * 1) prepare local config (defaults or caller override)
     * 2) probe/add I2C device
     * 3) verify part ID
     * 4) reset and configure registers
     * 5) flush FIFO and mark initialized
     */
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    max30102_config_t cfg = max30102_default_config();
    if (config != NULL) {
        cfg = *config;
    }

    /* Clear handle first so partial init does not leave stale fields. */
    memset(device, 0, sizeof(*device));
    device->timeout_ms = (cfg.timeout_ms == 0U) ? MAX30102_TIMEOUT_DEFAULT_MS : cfg.timeout_ms;
    device->i2c_dev.address = cfg.address;
    device->i2c_dev.scl_speed_hz = 400000U;
    device->sample_rate_hz = max30102_sample_rate_enum_to_hz(cfg.sample_rate);

    /* Probe confirms address ACK before full register configuration. */
    esp_err_t err = i2c_manager_probe(cfg.address, device->timeout_ms);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MAX30102 probe failed at 0x%02X (%s)", cfg.address, esp_err_to_name(err));
        return err;
    }

    err = i2c_manager_add_device(&device->i2c_dev);
    if (err != ESP_OK) {
        return err;
    }

    /* Read PART_ID to catch wrong wiring or unexpected sensor variant early. */
    uint8_t part_id = 0;
    err = max30102_read_reg(device, MAX30102_REG_PART_ID, &part_id);
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }
    if (part_id != MAX30102_PART_ID) {
        ESP_LOGW(TAG, "Unexpected MAX30102 PART_ID 0x%02X (expected 0x%02X)", part_id, MAX30102_PART_ID);
    }

    /* Configure device in a known state before writing FIFO/SPO2 settings. */
    err = max30102_reset(device);
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }

    /* Interrupts are optional for this polling-based driver but still configurable. */
    err = max30102_config_interrupts(device, cfg.interrupt_enable_1, cfg.interrupt_enable_2);
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }

    /* FIFO config bit-field: sample averaging, rollover mode, almost-full level. */
    const uint8_t fifo_cfg =
        (uint8_t)(((uint8_t)(cfg.sample_average & 0x07U) << 5) | ((cfg.fifo_rollover_enable ? 1U : 0U) << 4) |
                  (cfg.fifo_almost_full & 0x0FU));
    err = max30102_write_reg(device, MAX30102_REG_FIFO_CONFIG, fifo_cfg);
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }

    /* MODE_CONFIG lower bits select HR/SPO2 operating mode. */
    err = max30102_write_reg(device, MAX30102_REG_MODE_CONFIG, (uint8_t)(cfg.mode & 0x07U));
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }

    /*
     * SPO2 config bit-field:
     * [6:5] ADC full-scale range, [4:2] sample rate, [1:0] pulse width.
     */
    const uint8_t spo2_cfg = (uint8_t)(((uint8_t)(cfg.adc_range & 0x03U) << 5) |
                                       ((uint8_t)(cfg.sample_rate & 0x07U) << 2) |
                                       ((uint8_t)(cfg.pulse_width & 0x03U)));
    err = max30102_write_reg(device, MAX30102_REG_SPO2_CONFIG, spo2_cfg);
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }

    /* LED pulse amplitudes directly affect signal strength and power usage. */
    err = max30102_write_reg(device, MAX30102_REG_LED1_PA, cfg.led_red_pa);
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }
    err = max30102_write_reg(device, MAX30102_REG_LED2_PA, cfg.led_ir_pa);
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }
    err = max30102_write_reg(device, MAX30102_REG_PILOT_PA, cfg.led_pilot_pa);
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }

    /*
     * MULTI_LED slots are not used in this driver mode.
     * Force zero to keep slot behavior deterministic.
     */
    err = max30102_write_reg(device, MAX30102_REG_MULTI_LED_CTRL1, 0);
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }
    err = max30102_write_reg(device, MAX30102_REG_MULTI_LED_CTRL2, 0);
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }

    /* Optional status read to clear any stale interrupt flags after configuration. */
    uint8_t status_1 = 0;
    uint8_t status_2 = 0;
    (void)max30102_read_interrupt_status(device, &status_1, &status_2);

    err = max30102_fifo_flush(device);
    if (err != ESP_OK) {
        max30102_deinit(device);
        return err;
    }

    device->initialized = true;
    ESP_LOGI(TAG,
             "MAX30102 initialized at 0x%02X sr=%.0fHz avg=%u led_red=0x%02X led_ir=0x%02X",
             cfg.address,
             device->sample_rate_hz,
             (unsigned)cfg.sample_average,
             (unsigned)cfg.led_red_pa,
             (unsigned)cfg.led_ir_pa);
    return ESP_OK;
}

esp_err_t max30102_deinit(max30102_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Mark invalid before removing from bus so later calls fail fast. */
    device->initialized = false;
    return i2c_manager_remove_device(&device->i2c_dev);
}

esp_err_t max30102_fifo_read_sample(max30102_t *device, max30102_sample_t *sample)
{
    /*
     * FIFO read flow (beginner version):
     * 1) Read 0x04..0x06 to get write/read pointer state.
     * 2) Compute how many unread sample frames are available.
     * 3) If empty, return ESP_ERR_NOT_FOUND (not a hard bus failure).
     * 4) Read one full sample frame from 0x07 FIFO data port in burst mode.
     * 5) Unpack RED/IR raw ADC values (18-bit each) from 6 payload bytes.
     *
     * Important:
     * - FIFO contains raw ADC optical samples only.
     * - HR/SpO2 are computed later in software from many samples.
     */
    if (!max30102_is_ready(device) || (sample == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Read 3 bytes starting at 0x04: WR_PTR, OVF_COUNTER, RD_PTR. */
    uint8_t pointers[3];
    esp_err_t err =
        i2c_manager_read_reg(&device->i2c_dev, MAX30102_REG_FIFO_WR_PTR, pointers, sizeof(pointers), device->timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    /*
     * Pointers are 5-bit circular counters, so we keep only lower 5 bits.
     * - wr_ptr: where sensor will write next sample frame
     * - rd_ptr: where host will read next sample frame
     */
    const uint8_t wr_ptr = pointers[0] & 0x1FU;
    const uint8_t ovf_cnt = pointers[1] & 0x1FU;
    const uint8_t rd_ptr = pointers[2] & 0x1FU;

    /*
     * Available unread frames = (wr_ptr - rd_ptr) modulo 32.
     * This works because pointers wrap around in a circular FIFO.
     */
    const uint8_t available = (uint8_t)((wr_ptr - rd_ptr) & 0x1FU);
    if (available == 0U) {
        /* FIFO empty: caller can try again next cycle. */
        return ESP_ERR_NOT_FOUND;
    }

    /*
     * Overflow meaning:
     * - ovf_cnt > 0 indicates host read lag at least once.
     * - Older unread samples may have been overwritten before this read.
     *
     * Current code keeps behavior simple: still read latest available frame.
     */
    (void)ovf_cnt;

    /*
     * In SpO2 mode, one FIFO sample frame is exactly 6 bytes:
     * - RED channel: 3 bytes
     * - IR channel:  3 bytes
     *
     * Why read from 0x07 in burst mode:
     * - 0x07 is a FIFO data port, not a normal register with fixed content.
     * - Repeated reads from 0x07 return sequential FIFO bytes (stream behavior).
     * - It does NOT behave like a normal map where reads move to 0x08/0x09 payload.
     */
    uint8_t fifo_payload[6];
    err = i2c_manager_read_reg(&device->i2c_dev, MAX30102_REG_FIFO_DATA, fifo_payload, sizeof(fifo_payload), device->timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    /*
     * Byte-to-value unpacking:
     * - combine 3 bytes using shifts to form a temporary 24-bit value
     * - apply mask 0x03FFFF to keep only valid 18-bit ADC payload bits
     *
     * Channel order in SpO2 mode for this read:
     * - bytes[0..2] => RED raw ADC
     * - bytes[3..5] => IR  raw ADC
     */
    sample->red =
        (((uint32_t)fifo_payload[0] << 16) | ((uint32_t)fifo_payload[1] << 8) | fifo_payload[2]) & 0x3FFFFU;
    sample->ir =
        (((uint32_t)fifo_payload[3] << 16) | ((uint32_t)fifo_payload[4] << 8) | fifo_payload[5]) & 0x3FFFFU;
    sample->timestamp_ms = (uint32_t)esp_log_timestamp();

    /*
     * Raw sample stream for debugging:
     * - includes timestamp, sample index, RED/IR, last-known contact state.
     */
#if MAX30102_ENABLE_SAMPLE_LOG
    max30102_log_raw_sample(sample, s_sample_log_counter++, s_sample_log_contact);
#endif
    return ESP_OK;
}

esp_err_t max30102_read_die_temperature_c(max30102_t *device, float *temperature_c)
{
    /*
     * Die temperature read sequence from datasheet:
     * 1) trigger conversion
     * 2) poll temp-ready flag
     * 3) read integer + fractional nibble registers
     */
    if (!max30102_is_ready(device) || (temperature_c == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = max30102_write_reg(device, MAX30102_REG_TEMP_CONFIG, MAX30102_TEMP_START_CONVERSION_BIT);
    if (err != ESP_OK) {
        return err;
    }

    /* Poll in short intervals to avoid long blocking delay. */
    for (int i = 0; i < 6; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
        uint8_t int_status_2 = 0;
        err = max30102_read_reg(device, MAX30102_REG_INTR_STATUS_2, &int_status_2);
        if (err != ESP_OK) {
            return err;
        }
        if ((int_status_2 & MAX30102_TEMP_RDY_INT_BIT) != 0U) {
            break;
        }
        if (i == 5) {
            return ESP_ERR_TIMEOUT;
        }
    }

    uint8_t temp_int = 0;
    uint8_t temp_frac = 0;
    err = max30102_read_reg(device, MAX30102_REG_TEMP_INT, &temp_int);
    if (err != ESP_OK) {
        return err;
    }
    err = max30102_read_reg(device, MAX30102_REG_TEMP_FRAC, &temp_frac);
    if (err != ESP_OK) {
        return err;
    }

    /*
     * Fraction register uses 1/16 C steps in low nibble.
     * 0.0625 = 1/16.
     */
    *temperature_c = (float)((int8_t)temp_int) + ((float)(temp_frac & 0x0FU) * 0.0625f);
    return ESP_OK;
}

void max30102_algo_init(max30102_algo_state_t *state, float sample_rate_hz)
{
    if (state == NULL) {
        return;
    }

    /* Zero all history so stale peaks/filters do not survive across sessions. */
    memset(state, 0, sizeof(*state));
    /*
     * Use caller sample rate when valid; otherwise fallback to compile-time default.
     * This keeps timing math sane even if caller passes 0 by mistake.
     */
    state->sample_rate_hz = (sample_rate_hz > 0.0f) ? sample_rate_hz : (float)MAX30102_SAMPLE_RATE;
    state->contact_quality = MAX30102_CONTACT_NO_FINGER;
    state->settling_until_ms = 0U;
    state->contact_transition_ms = 0U;
    state->valid_candidate_start_ms = 0U;
    state->valid_bad_sample_streak = 0U;
    max30102_algo_reset_tracking(state, 0.0f, 0.0f, 0U, "init");
    state->contact_quality = MAX30102_CONTACT_NO_FINGER;
}

void max30102_algo_update(max30102_algo_state_t *state, const max30102_sample_t *sample, max30102_processed_t *out)
{
    /*
     * Main per-sample algorithm entry.
     *
     * High-level stages:
     * 1) DC removal + AC filtering
     * 2) signal quality metrics (RMS, motion, saturation)
     * 3) contact-state machine + settling control
     * 4) beat detection + HR estimate
     * 5) SpO2 ratio estimate
     * 6) confidence/validity gating + hold behavior
     * 7) optional debug logs + output snapshot
     */
    if ((state == NULL) || (sample == NULL)) {
        return;
    }

    const float red = (float)sample->red;
    const float ir = (float)sample->ir;

    /*
     * Bootstrap filter state on first sample.
     * Without this, first few outputs can jump due to uninitialized baseline.
     */
    if (state->sample_count == 0U) {
        state->red_dc = red;
        state->ir_dc = ir;
        state->ir_ac_baseline = 0.0f;
        state->prev_prev_ir_ac = 0.0f;
        state->prev_ir_ac = 0.0f;
    }

    /* EMA low-pass for DC, plus faster EMA on AC residual for pulse waveform. */
    const float dc_alpha = 0.02f;
    const float ac_alpha = 0.22f;
    state->red_dc += dc_alpha * (red - state->red_dc);
    state->ir_dc += dc_alpha * (ir - state->ir_dc);

    const float red_ac = red - state->red_dc;
    const float ir_ac = ir - state->ir_dc;
    state->red_ac_filtered += ac_alpha * (red_ac - state->red_ac_filtered);
    state->ir_ac_filtered += ac_alpha * (ir_ac - state->ir_ac_filtered);
    state->ir_ac_baseline += MAX30102_DET_BASELINE_ALPHA * (state->ir_ac_filtered - state->ir_ac_baseline);
    const float det_curr = state->ir_ac_filtered - state->ir_ac_baseline;
    const float ir_deriv = det_curr - state->prev_ir_ac;
    state->ir_deriv_abs_ema += 0.12f * (fabsf(ir_deriv) - state->ir_deriv_abs_ema);

    max30102_push_ac_sample(state, state->red_ac_filtered, state->ir_ac_filtered);
    state->sample_count++;

    float red_rms =
        (state->ac_window_count > 0U) ? sqrtf(fmaxf(state->red_ac2_sum / (float)state->ac_window_count, 0.0f)) : 0.0f;
    float ir_rms =
        (state->ac_window_count > 0U) ? sqrtf(fmaxf(state->ir_ac2_sum / (float)state->ac_window_count, 0.0f)) : 0.0f;

    /*
     * Motion metric:
     * - numerator: EMA of IR derivative magnitude
     * - denominator: AC RMS magnitude
     * Large ratio usually means noisy/motion-corrupted waveform.
     */
    state->motion_metric = state->ir_deriv_abs_ema / fmaxf(ir_rms, 1.0f);
    state->signal_saturated = (red >= MAX30102_SIGNAL_SATURATION_LEVEL) || (ir >= MAX30102_SIGNAL_SATURATION_LEVEL);

    /*
     * Contact state machine:
     * - raw IR/DC level is the primary gate
     * - AC/motion quality is a secondary qualifier
     * - hysteresis/debounce avoids weak<->valid chatter
     */
    const max30102_contact_quality_t prev_contact = state->contact_quality;
    const bool finger_lost = (ir < MAX30102_IR_FINGER_OFF_THRESHOLD) ||
                             (state->ir_dc < (MAX30102_IR_FINGER_OFF_THRESHOLD * 0.90f));
    const bool settling_active = (prev_contact == MAX30102_CONTACT_VALID) && (state->settling_until_ms != 0U) &&
                                 max30102_time_is_before(sample->timestamp_ms, state->settling_until_ms);

    max30102_contact_quality_t observed_contact = prev_contact;
    /*
     * Contact-state logic:
     * - hard no-contact immediately when IR falls below finger-off threshold
     * - valid->weak demotion requires multiple bad samples (hysteresis)
     * - weak->valid promotion requires stable-good duration (debounce)
     */
    if (finger_lost) {
        observed_contact = MAX30102_CONTACT_NO_FINGER;
        state->valid_candidate_start_ms = 0U;
        state->valid_bad_sample_streak = 0U;
    } else if (prev_contact == MAX30102_CONTACT_VALID) {
        if (settling_active) {
            observed_contact = MAX30102_CONTACT_VALID;
            state->valid_bad_sample_streak = 0U;
        } else {
            const bool primary_bad =
                (state->ir_dc < MAX30102_VALID_EXIT_IR_DC_THRESHOLD) || (ir < MAX30102_VALID_EXIT_IR_RAW_THRESHOLD);
            const bool quality_bad = state->signal_saturated || (ir_rms < MAX30102_VALID_EXIT_AC_MIN) ||
                                     (ir_rms > MAX30102_SIGNAL_AC_MAX) || (red_rms > MAX30102_SIGNAL_AC_MAX) ||
                                     (state->motion_metric > MAX30102_VALID_EXIT_MOTION_RATIO);

            if (primary_bad || quality_bad) {
                if (state->valid_bad_sample_streak < UINT16_MAX) {
                    state->valid_bad_sample_streak++;
                }
            } else {
                state->valid_bad_sample_streak = 0U;
            }

            observed_contact =
                (state->valid_bad_sample_streak >= MAX30102_VALID_EXIT_BAD_SAMPLES) ? MAX30102_CONTACT_WEAK :
                                                                                      MAX30102_CONTACT_VALID;
        }
    } else {
        const bool primary_good =
            (ir >= MAX30102_IR_FINGER_THRESHOLD) || (state->ir_dc >= MAX30102_VALID_ENTER_IR_DC_THRESHOLD);
        const bool secondary_good =
            !state->signal_saturated && (ir_rms >= MAX30102_VALID_ENTER_AC_MIN) && (ir_rms <= MAX30102_SIGNAL_AC_MAX) &&
            (red_rms <= MAX30102_SIGNAL_AC_MAX) && (state->motion_metric <= (MAX30102_MOTION_NOISE_RATIO_MAX * 1.20f));

        observed_contact = MAX30102_CONTACT_WEAK;
        state->valid_bad_sample_streak = 0U;
        if (primary_good && secondary_good) {
            if (state->valid_candidate_start_ms == 0U) {
                state->valid_candidate_start_ms = sample->timestamp_ms;
            }
            const uint32_t stable_ms = sample->timestamp_ms - state->valid_candidate_start_ms;
            if (stable_ms >= MAX30102_VALID_ENTER_STABLE_MS) {
                observed_contact = MAX30102_CONTACT_VALID;
            }
        } else {
            state->valid_candidate_start_ms = 0U;
        }
    }

    const bool contact_transition = (observed_contact != prev_contact);
    bool in_settling = false;
    /*
     * On transitions we log state changes and reset only when entering/leaving
     * no-contact. We intentionally do NOT full-reset on weak<->valid chatter.
     */
    if (contact_transition) {
        state->contact_transition_ms = sample->timestamp_ms;
        MAX30102_EVT_LOGI(
                 "contact transition: %s -> %s (IRraw=%.0f IRdc=%.0f IRac=%.1f bad=%u)",
                 max30102_contact_quality_to_cstr(prev_contact),
                 max30102_contact_quality_to_cstr(observed_contact),
                 ir,
                 state->ir_dc,
                 ir_rms,
                 (unsigned)state->valid_bad_sample_streak);

        if (((prev_contact == MAX30102_CONTACT_NO_FINGER) && (observed_contact != MAX30102_CONTACT_NO_FINGER)) ||
            ((prev_contact != MAX30102_CONTACT_NO_FINGER) && (observed_contact == MAX30102_CONTACT_NO_FINGER))) {
            const bool to_no_contact = (observed_contact == MAX30102_CONTACT_NO_FINGER);
            /*
             * Hard reset on no_contact:
             * - zero seeds ensure previous-session DC/baseline values do not survive.
             *
             * Re-entry from no_contact:
             * - seed from current raw sample so filters can lock quickly.
             */
            max30102_algo_reset_tracking(state,
                                         to_no_contact ? 0.0f : red,
                                         to_no_contact ? 0.0f : ir,
                                         sample->timestamp_ms,
                                         to_no_contact ? "finger_removed" : "finger_detected");
            red_rms = 0.0f;
            ir_rms = 0.0f;
        }

        if ((prev_contact != MAX30102_CONTACT_VALID) && (observed_contact == MAX30102_CONTACT_VALID)) {
            state->settling_until_ms = sample->timestamp_ms + MAX30102_SETTLING_TIME_MS;
            state->valid_candidate_start_ms = 0U;
            MAX30102_EVT_LOGI(
                     "valid contact settling started for %u ms (until=%" PRIu32 ")",
                     (unsigned)MAX30102_SETTLING_TIME_MS,
                     state->settling_until_ms);
        } else if (observed_contact == MAX30102_CONTACT_NO_FINGER) {
            state->settling_until_ms = 0U;
        }
    }

    state->contact_quality = observed_contact;
    if ((state->contact_quality == MAX30102_CONTACT_VALID) && (state->settling_until_ms != 0U)) {
        if (max30102_time_is_before(sample->timestamp_ms, state->settling_until_ms)) {
            in_settling = true;
        } else {
            state->settling_until_ms = 0U;
            MAX30102_EVT_LOGI("valid contact settling complete");
        }
    }

    /* Share latest contact state with FIFO logger so each raw sample line includes context. */
    s_sample_log_contact = state->contact_quality;

    /*
     * Beat timing constraints in sample domain:
     * - min_gap corresponds to MAX HR
     * - max_gap corresponds to MIN HR
     */
    const float min_samples_f = fmaxf(state->sample_rate_hz * MAX30102_MIN_ESTIMATION_SECONDS, 30.0f);
    const uint32_t min_samples = (uint32_t)min_samples_f;
    const uint32_t min_gap = (uint32_t)(state->sample_rate_hz * 60.0f / MAX30102_MAX_HR_BPM);
    const uint32_t max_gap = (uint32_t)(state->sample_rate_hz * 60.0f / MAX30102_MIN_HR_BPM);
    float peak_threshold = fmaxf(
        fmaxf(ir_rms * MAX30102_PEAK_THRESHOLD_RMS_SCALE, state->ir_deriv_abs_ema * MAX30102_PEAK_THRESHOLD_DERIV_SCALE),
        MAX30102_PEAK_THRESHOLD_MIN);
    /*
     * Bootstrap easing:
     * allow the first few beats to lock faster before tightening detection.
     */
    if (state->beat_count < (uint8_t)MAX30102_MIN_VALID_BEATS) {
        peak_threshold = fmaxf(peak_threshold * MAX30102_PEAK_BOOTSTRAP_THRESHOLD_SCALE, MAX30102_PEAK_THRESHOLD_MIN);
    }
    state->last_peak_threshold = peak_threshold;
    bool beat_detected = false;
    uint16_t beat_interval_ms = 0U;

    const float det_prev_prev = state->prev_prev_ir_ac;
    const float det_prev = state->prev_ir_ac;

    /* Beat detection runs only for stable valid contact after settling window. */
    if ((state->sample_count >= 2U) && (state->contact_quality == MAX30102_CONTACT_VALID) && !in_settling) {
        const uint32_t candidate_idx = state->sample_count - 1U;

        /*
         * Detect both maxima and minima:
         * - Some optical setups produce stronger troughs than peaks.
         * - We lock to one polarity once a stream starts to avoid double-counting.
         */
        /*
         * Allow slight plateaus at extrema by accepting one non-strict side.
         * This helps with flat-topped/flat-bottomed pulses.
         */
        const bool local_max =
            ((det_prev >= det_prev_prev) && (det_prev > det_curr)) || ((det_prev > det_prev_prev) && (det_prev >= det_curr));
        const bool local_min =
            ((det_prev <= det_prev_prev) && (det_prev < det_curr)) || ((det_prev < det_prev_prev) && (det_prev <= det_curr));

        if (local_max || local_min) {
            state->peak_candidate_count++;

            const int8_t candidate_polarity = local_max ? 1 : -1;
            const float candidate_amp = local_max ? det_prev : -det_prev;
            const float prominence = local_max ? (det_prev - fminf(det_prev_prev, det_curr))
                                               : (fmaxf(det_prev_prev, det_curr) - det_prev);
            const float slope = fmaxf(fabsf(det_prev - det_prev_prev), fabsf(det_prev - det_curr));
            const float prom_min =
                fmaxf(MAX30102_PEAK_PROMINENCE_ABS_MIN, fmaxf(MAX30102_PEAK_PROMINENCE_MIN, peak_threshold * MAX30102_PEAK_PROMINENCE_RATIO_MIN));
            const float slope_min = fmaxf(MAX30102_PEAK_SLOPE_ABS_MIN, peak_threshold * MAX30102_PEAK_SLOPE_RATIO_MIN);

            if (candidate_amp < peak_threshold) {
                state->peak_reject_shape_count++;
                ESP_LOGD(TAG,
                         "peak reject: low_amp amp=%.2f thr=%.2f prom=%.2f slope=%.2f pol=%d",
                         candidate_amp,
                         peak_threshold,
                         prominence,
                         slope,
                         (int)candidate_polarity);
            } else if ((prominence < prom_min) && (slope < slope_min)) {
                state->peak_reject_shape_count++;
                ESP_LOGD(TAG,
                         "peak reject: low_shape amp=%.2f thr=%.2f prom=%.2f(min=%.2f) slope=%.2f(min=%.2f) pol=%d",
                         candidate_amp,
                         peak_threshold,
                         prominence,
                         prom_min,
                         slope,
                         slope_min,
                         (int)candidate_polarity);
            } else if (state->motion_metric > (MAX30102_MOTION_NOISE_RATIO_MAX * 1.5f)) {
                state->peak_reject_noise_count++;
                ESP_LOGD(TAG,
                         "peak reject: noisy motion=%.2f amp=%.2f thr=%.2f prom=%.2f slope=%.2f",
                         state->motion_metric,
                         candidate_amp,
                         peak_threshold,
                         prominence,
                         slope);
            } else {
                const bool polarity_ok = (state->beat_polarity == 0) || (state->beat_polarity == candidate_polarity);
                if (!polarity_ok) {
                    state->peak_reject_polarity_count++;
                    ESP_LOGD(TAG,
                             "peak reject: polarity lock=%d cand=%d amp=%.2f thr=%.2f",
                             (int)state->beat_polarity,
                             (int)candidate_polarity,
                             candidate_amp,
                             peak_threshold);
                } else if (state->last_peak_index < 0) {
                    state->last_peak_index = (int32_t)candidate_idx;
                    state->beat_polarity = candidate_polarity;
                    MAX30102_EVT_LOGI(
                             "peak prime: pol=%d amp=%.2f thr=%.2f prom=%.2f idx=%" PRIu32,
                             (int)candidate_polarity,
                             candidate_amp,
                             peak_threshold,
                             prominence,
                             candidate_idx);
                } else {
                    const uint32_t delta = candidate_idx - (uint32_t)state->last_peak_index;
                    if (delta < min_gap) {
                        state->peak_reject_timing_count++;
                        ESP_LOGD(TAG,
                                 "peak reject: too_close dt=%" PRIu32 " min=%" PRIu32 " amp=%.2f",
                                 delta,
                                 min_gap,
                                 candidate_amp);
                    } else if (delta > max_gap) {
                        state->peak_reject_timing_count++;
                        /* Re-prime with current extremum when previous anchor is stale. */
                        state->last_peak_index = (int32_t)candidate_idx;
                        state->beat_polarity = candidate_polarity;
                        ESP_LOGD(TAG,
                                 "peak reprime: stale dt=%" PRIu32 " max=%" PRIu32 " pol=%d",
                                 delta,
                                 max_gap,
                                 (int)candidate_polarity);
                    } else {
                        max30102_push_interval(state, (uint16_t)delta);
                        state->last_peak_interval_samples = (uint16_t)delta;
                        state->peak_count++;
                        state->last_accepted_peak_index = (int32_t)candidate_idx;
                        state->raw_heart_rate_bpm = 60.0f * (state->sample_rate_hz / (float)delta);
                        beat_interval_ms = (uint16_t)((1000.0f * (float)delta) / state->sample_rate_hz);
                        beat_detected = true;
                        state->last_peak_index = (int32_t)candidate_idx;
                        state->beat_polarity = candidate_polarity;
                        MAX30102_EVT_LOGI(
                                 "peak accept cnt=%" PRIu32 " dt=%ums rawHR=%.1f pol=%d amp=%.2f thr=%.2f prom=%.2f motion=%.2f",
                                 state->peak_count,
                                 (unsigned)beat_interval_ms,
                                 state->raw_heart_rate_bpm,
                                 (int)candidate_polarity,
                                 candidate_amp,
                                 peak_threshold,
                                 prominence,
                                 state->motion_metric);
                    }
                }
            }
        }
    }

    state->prev_prev_ir_ac = state->prev_ir_ac;
    state->prev_ir_ac = det_curr;

    bool new_hr_valid = false;
    bool new_spo2_valid = false;
    float new_hr_bpm = state->heart_rate_bpm;
    float new_spo2 = state->spo2_percent;
    bool hr_stale_timeout = false;
    /*
     * Default to \"insufficient samples\" each cycle and overwrite with specific
     * reasons as evidence becomes available.
     */
    state->hr_reason = MAX30102_REASON_INSUFFICIENT_SAMPLES;
    state->spo2_reason = MAX30102_REASON_INSUFFICIENT_SAMPLES;
    state->heart_rate_confidence = 0.0f;
    state->spo2_confidence = 0.0f;
    state->last_ratio = NAN;
    state->raw_spo2_percent = NAN;

    /*
     * Reason hierarchy:
     * - saturation/no-contact/weak-contact take priority
     * - only when valid contact + enough samples do we compute estimates
     */
    if (state->signal_saturated) {
        state->hr_reason = MAX30102_REASON_SIGNAL_SATURATED;
        state->spo2_reason = MAX30102_REASON_SIGNAL_SATURATED;
    } else if (state->contact_quality == MAX30102_CONTACT_NO_FINGER) {
        state->hr_reason = MAX30102_REASON_NO_CONTACT;
        state->spo2_reason = MAX30102_REASON_NO_CONTACT;
    } else if (state->contact_quality == MAX30102_CONTACT_WEAK) {
        if ((ir_rms > MAX30102_SIGNAL_AC_MAX) || (red_rms > MAX30102_SIGNAL_AC_MAX) ||
            (state->motion_metric > MAX30102_MOTION_NOISE_RATIO_MAX)) {
            state->hr_reason = MAX30102_REASON_SIGNAL_NOISY;
            state->spo2_reason = MAX30102_REASON_SIGNAL_NOISY;
        } else {
            state->hr_reason = MAX30102_REASON_WEAK_CONTACT;
            state->spo2_reason = MAX30102_REASON_WEAK_CONTACT;
        }
    } else if (in_settling) {
        state->hr_reason = MAX30102_REASON_INSUFFICIENT_SAMPLES;
        state->spo2_reason = MAX30102_REASON_INSUFFICIENT_SAMPLES;
    } else if (state->sample_count < min_samples) {
        state->hr_reason = MAX30102_REASON_INSUFFICIENT_SAMPLES;
        state->spo2_reason = MAX30102_REASON_INSUFFICIENT_SAMPLES;
    } else {
        const uint32_t hr_stale_samples =
            (uint32_t)fmaxf((state->sample_rate_hz * MAX30102_HR_STALE_SECONDS), (float)max_gap);
        const uint32_t samples_since_last_beat = max30102_samples_since_last_accepted_beat(state);
        const bool hr_has_recent_beat =
            (samples_since_last_beat != UINT32_MAX) && (samples_since_last_beat <= hr_stale_samples);

        float hr_bpm = 0.0f;
        float hr_conf = 0.0f;
        /* HR path based on beat-interval history, only when recent beats exist. */
        if (!hr_has_recent_beat) {
            hr_stale_timeout = true;
            state->hr_reason = MAX30102_REASON_INSUFFICIENT_SAMPLES;
            state->heart_rate_confidence = 0.0f;
            state->raw_heart_rate_bpm = NAN;
            if (state->beat_count > 0U) {
                memset(state->beat_intervals, 0, sizeof(state->beat_intervals));
                state->beat_count = 0U;
                state->beat_head = 0U;
            }
            state->last_accepted_peak_index = -1;
        } else if (max30102_estimate_heart_rate(state, &hr_bpm, &hr_conf)) {
            state->raw_heart_rate_bpm = hr_bpm;
            state->heart_rate_confidence = hr_conf;
            if (hr_conf >= MAX30102_MIN_CONFIDENCE) {
                new_hr_bpm = hr_bpm;
                new_hr_valid = true;
                state->hr_reason = MAX30102_REASON_NONE;
            } else {
                printf("Heart rate unstable (conf: %f)\n", hr_conf);
                state->hr_reason = MAX30102_REASON_UNSTABLE_BEATS;
            }
        } else {
            state->hr_reason = MAX30102_REASON_UNSTABLE_BEATS;
        }

        /* SpO2 ratio uses both RED and IR AC/DC terms. */
        if ((red_rms < MAX30102_SIGNAL_AC_MIN) || (ir_rms < MAX30102_SIGNAL_AC_MIN)) {
            state->spo2_reason = MAX30102_REASON_SIGNAL_WEAK;
        } else if ((state->red_dc <= 1.0f) || (state->ir_dc <= 1.0f) || (ir_rms <= 1.0f)) {
            state->spo2_reason = MAX30102_REASON_INVALID_RATIO;
        } else {
            /*
             * SpO2 ratio-of-ratios:
             * R = (AC_red / DC_red) / (AC_ir / DC_ir)
             * Then mapped by linear prototype formula to SpO2 estimate.
             */
            const float ratio = (red_rms / state->red_dc) / (ir_rms / state->ir_dc);
            state->last_ratio = ratio;
            if (!isfinite(ratio) || (ratio < 0.2f) || (ratio > 2.2f)) {
                state->spo2_reason = MAX30102_REASON_INVALID_RATIO;
            } else {
                float spo2 = 110.0f - (25.0f * ratio);
                spo2 = max30102_clampf(spo2, 70.0f, 100.0f);
                state->raw_spo2_percent = spo2;

                const float ac_strength = max30102_clampf((fminf(red_rms, ir_rms) - MAX30102_SIGNAL_AC_MIN) /
                                                              (MAX30102_SIGNAL_AC_MAX - MAX30102_SIGNAL_AC_MIN),
                                                          0.0f,
                                                          1.0f);
                const float ratio_conf = 1.0f - max30102_clampf(fabsf(ratio - 0.9f) / 1.2f, 0.0f, 1.0f);
                const float motion_conf = 1.0f - max30102_clampf((state->motion_metric - 1.0f) / 3.0f, 0.0f, 1.0f);
                const float beat_support = fmaxf(state->heart_rate_confidence, (state->beat_count >= 2U) ? 0.45f : 0.15f);
                state->spo2_confidence = max30102_clampf(
                    (0.35f * ac_strength) + (0.25f * ratio_conf) + (0.20f * motion_conf) + (0.20f * beat_support),
                    0.0f,
                    1.0f);

                if (state->spo2_confidence >= (MAX30102_MIN_CONFIDENCE * 0.90f)) {
                    new_spo2 = spo2;
                    new_spo2_valid = true;
                    state->spo2_reason = MAX30102_REASON_NONE;
                } else {
                    state->spo2_reason = MAX30102_REASON_UNSTABLE_BEATS;
                }
            }
        }
    }

    /*
     * Hold behavior:
     * - keep last valid value briefly during non-severe invalid conditions
     * - drop immediately on severe faults (no contact / saturation)
     */
    const uint16_t hold_limit = (uint16_t)fmaxf(state->sample_rate_hz * MAX30102_VALUE_HOLD_SECONDS, 20.0f);
    const bool severe_invalid =
        (state->hr_reason == MAX30102_REASON_NO_CONTACT) || (state->hr_reason == MAX30102_REASON_SIGNAL_SATURATED) ||
        hr_stale_timeout;
    if (new_hr_valid) {
        state->heart_rate_bpm =
            state->heart_rate_valid ? ((0.70f * state->heart_rate_bpm) + (0.30f * new_hr_bpm)) : new_hr_bpm;
        state->heart_rate_valid = true;
        state->hr_hold_count = 0U;
        state->hr_reason = MAX30102_REASON_NONE;
    } else if (severe_invalid) {
        state->heart_rate_valid = false;
        state->hr_hold_count = hold_limit;
    } else if (state->heart_rate_valid && (state->hr_hold_count < hold_limit)) {
        state->hr_hold_count++;
        state->hr_reason = MAX30102_REASON_NONE;
    } else {
        state->heart_rate_valid = false;
    }

    const bool severe_spo2_invalid = (state->spo2_reason == MAX30102_REASON_NO_CONTACT) ||
                                     (state->spo2_reason == MAX30102_REASON_SIGNAL_SATURATED);
    if (new_spo2_valid) {
        state->spo2_percent = state->spo2_valid ? ((0.80f * state->spo2_percent) + (0.20f * new_spo2)) : new_spo2;
        state->spo2_valid = true;
        state->spo2_hold_count = 0U;
        state->spo2_reason = MAX30102_REASON_NONE;
    } else if (severe_spo2_invalid) {
        state->spo2_valid = false;
        state->spo2_hold_count = hold_limit;
    } else if (state->spo2_valid && (state->spo2_hold_count < hold_limit)) {
        state->spo2_hold_count++;
        state->spo2_reason = MAX30102_REASON_NONE;
    } else {
        state->spo2_valid = false;
    }
#if MAX30102_ENABLE_ALGO_PERIODIC_LOG
    /*
     * Throttled debug summary log.
     * This is intentionally verbose for tuning and failure analysis.
     */
    if ((sample->timestamp_ms - state->last_debug_log_ms) >= MAX30102_DEBUG_LOG_INTERVAL_MS) {
        state->last_debug_log_ms = sample->timestamp_ms;
        const uint32_t settle_left_ms =
            max30102_time_is_before(sample->timestamp_ms, state->settling_until_ms)
                ? (state->settling_until_ms - sample->timestamp_ms)
                : 0U;
        const uint32_t candidate_ms =
            (state->valid_candidate_start_ms == 0U) ? 0U : (sample->timestamp_ms - state->valid_candidate_start_ms);
        ESP_LOGI(TAG,
                 "algo c=%s settle=%ums cand=%ums bad=%u ir_raw=%.0f red_raw=%.0f dc_ir=%.0f "
                 "det_base=%.2f det_prev=%.2f det_cur=%.2f ac_ir=%.1f thr=%.2f pol=%d "
                 "candPk=%" PRIu32 " rej[t=%" PRIu32 " s=%" PRIu32 " n=%" PRIu32 " p=%" PRIu32 "] "
                 "peaks=%" PRIu32 " beats=%u beat=%s int=%u rawHR=%.1f HR=%.1f(%s,%.2f) motion=%.2f "
                 "ratio=%.3f rawSpO2=%.1f SpO2=%.1f(%s,%.2f)",
                 max30102_contact_quality_to_cstr(state->contact_quality),
                 settle_left_ms,
                 candidate_ms,
                 (unsigned)state->valid_bad_sample_streak,
                 ir,
                 red,
                 state->ir_dc,
                 state->ir_ac_baseline,
                 det_prev,
                 det_curr,
                 ir_rms,
                 state->last_peak_threshold,
                 (int)state->beat_polarity,
                 state->peak_candidate_count,
                 state->peak_reject_timing_count,
                 state->peak_reject_shape_count,
                 state->peak_reject_noise_count,
                 state->peak_reject_polarity_count,
                 state->peak_count,
                 (unsigned)state->beat_count,
                 beat_detected ? "yes" : "no",
                 (unsigned)beat_interval_ms,
                 state->raw_heart_rate_bpm,
                 state->heart_rate_bpm,
                 max30102_reason_to_cstr(state->hr_reason),
                 state->heart_rate_confidence,
                 state->motion_metric,
                 state->last_ratio,
                 state->raw_spo2_percent,
                 state->spo2_percent,
                 max30102_reason_to_cstr(state->spo2_reason),
                 state->spo2_confidence);
    }
#endif

    /* Copy current snapshot to caller-facing output struct if requested. */
    if (out != NULL) {
        out->red_raw = sample->red;
        out->ir_raw = sample->ir;
        out->red_filtered = state->red_dc + state->red_ac_filtered;
        out->ir_filtered = state->ir_dc + state->ir_ac_filtered;
        out->red_dc = state->red_dc;
        out->ir_dc = state->ir_dc;
        out->red_ac_rms = red_rms;
        out->ir_ac_rms = ir_rms;
        out->heart_rate_bpm = state->heart_rate_bpm;
        out->spo2_percent = state->spo2_percent;
        out->heart_rate_confidence = state->heart_rate_confidence;
        out->spo2_confidence = state->spo2_confidence;
        out->raw_heart_rate_bpm = state->raw_heart_rate_bpm;
        out->raw_spo2_percent = state->raw_spo2_percent;
        out->ratio = state->last_ratio;
        out->peak_threshold = state->last_peak_threshold;
        out->motion_metric = state->motion_metric;
        out->beat_interval_ms = beat_interval_ms;
        out->beat_detected = beat_detected;
        out->contact_quality = state->contact_quality;
        out->hr_reason = state->hr_reason;
        out->spo2_reason = state->spo2_reason;
        out->signal_saturated = state->signal_saturated;
        out->heart_rate_valid = state->heart_rate_valid;
        out->spo2_valid = state->spo2_valid;
    }
}

const char *max30102_contact_quality_to_string(max30102_contact_quality_t quality)
{
    /* Public wrapper around internal enum-to-string helper. */
    return max30102_contact_quality_to_cstr(quality);
}

const char *max30102_reason_to_string(max30102_reason_t reason)
{
    /* Public wrapper around internal enum-to-string helper. */
    return max30102_reason_to_cstr(reason);
}
