#ifndef MAX30102_H
#define MAX30102_H

/*
 * MAX30102 driver (ESP-IDF, I2C)
 *
 * Purpose:
 * - Talk to the MAX30102 optical sensor over I2C.
 * - Read raw RED/IR samples from FIFO.
 * - Provide a lightweight HR/SpO2 processing path for wearable prototyping.
 *
 * Important note for beginners:
 * - The algorithm in this file is a practical prototype implementation.
 * - It is useful for trend tracking and debugging sensor quality.
 * - It is NOT a medical-grade algorithm.
 *
 * Typical usage order:
 * 1) max30102_init(...)
 * 2) max30102_algo_init(...)
 * 3) Loop:
 *    - max30102_fifo_read_sample(...)
 *    - max30102_algo_update(...)
 * 4) Optional cleanup: max30102_deinit(...)
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "i2c_manager.h"

/*
 * Contact gate thresholds (IR channel).
 *
 * Why these are needed:
 * - The sensor can read values even without a finger/skin.
 * - We use IR magnitude to infer if there is usable optical contact.
 * - Contact state is then used to allow/reject HR/SpO2 output.
 */
#ifndef MAX30102_IR_FINGER_THRESHOLD
#define MAX30102_IR_FINGER_THRESHOLD 12000.0f
#endif

#ifndef MAX30102_IR_WEAK_CONTACT_THRESHOLD
#define MAX30102_IR_WEAK_CONTACT_THRESHOLD 25000.0f
#endif

#ifndef MAX30102_IR_FINGER_OFF_THRESHOLD
#define MAX30102_IR_FINGER_OFF_THRESHOLD 8000.0f
#endif

/*
 * Settling window after valid contact is detected.
 *
 * Why this exists:
 * - Immediately after finger placement, signal often has transients.
 * - If we compute HR/SpO2 during this transient, results are unstable.
 */
#ifndef MAX30102_SETTLING_TIME_MS
#define MAX30102_SETTLING_TIME_MS 1800U
#endif

/*
 * Raw sample logging controls.
 *
 * MAX30102_ENABLE_SAMPLE_LOG:
 * - 1: emit one log entry per sample
 * - 0: disable sample stream logging
 *
 * MAX30102_SAMPLE_LOG_TO_FILE:
 * - 1: append logs to MAX30102_SAMPLE_LOG_FILE_PATH using stdio
 * - 0: print with ESP_LOGI (recommended default on MCU targets)
 */
#ifndef MAX30102_ENABLE_SAMPLE_LOG
#define MAX30102_ENABLE_SAMPLE_LOG 0
#endif

#ifndef MAX30102_SAMPLE_LOG_TO_FILE
#define MAX30102_SAMPLE_LOG_TO_FILE 0
#endif

#ifndef MAX30102_SAMPLE_LOG_FILE_PATH
#define MAX30102_SAMPLE_LOG_FILE_PATH "max30102.log"
#endif

/*
 * Optional verbose event logs (contact transitions, peak accept, resets).
 * Keep disabled by default for better runtime performance and cleaner serial output.
 */
#ifndef MAX30102_ENABLE_EVENT_LOG
#define MAX30102_ENABLE_EVENT_LOG 0
#endif

/*
 * Optional periodic algorithm summary log (very verbose).
 * Disable by default to avoid repeated console traffic every interval.
 */
#ifndef MAX30102_ENABLE_ALGO_PERIODIC_LOG
#define MAX30102_ENABLE_ALGO_PERIODIC_LOG 0
#endif

/*
 * Sensor configuration defaults.
 *
 * LED current registers are 8-bit values from datasheet tables.
 * These defaults are tuned for easier finger detection on common modules.
 */
#ifndef MAX30102_LED_RED_CURRENT
#define MAX30102_LED_RED_CURRENT 0x45U
#endif

#ifndef MAX30102_LED_IR_CURRENT
#define MAX30102_LED_IR_CURRENT 0x50U
#endif

/* Default algorithm sample-rate assumption if caller passes invalid rate. */
#ifndef MAX30102_SAMPLE_RATE
#define MAX30102_SAMPLE_RATE 100U
#endif

/* FIFO averaging default used in max30102_default_config(). */
#ifndef MAX30102_SAMPLE_AVERAGING
#define MAX30102_SAMPLE_AVERAGING MAX30102_SAMPLE_AVG_16
#endif

/*
 * Algorithm buffer sizes.
 *
 * MAX30102_BUFFER_SIZE:
 * - RMS/AC rolling window length used for SpO2 and quality metrics.
 *
 * MAX30102_BEAT_HISTORY_SIZE:
 * - number of recent beat intervals used to estimate HR stability.
 */
#ifndef MAX30102_BUFFER_SIZE
#define MAX30102_BUFFER_SIZE 240U
#endif

#ifndef MAX30102_BEAT_HISTORY_SIZE
#define MAX30102_BEAT_HISTORY_SIZE 12U
#endif

/*
 * Validity gates.
 *
 * MAX30102_MIN_VALID_BEATS:
 * - minimum accepted beat interval count to estimate HR.
 *
 * MAX30102_MIN_CONFIDENCE:
 * - minimum confidence gate for publishing values as valid.
 */
#ifndef MAX30102_MIN_VALID_BEATS
#define MAX30102_MIN_VALID_BEATS 1U
// #define MAX30102_MIN_VALID_BEATS 2U

#endif

#ifndef MAX30102_MIN_CONFIDENCE
// #define MAX30102_MIN_CONFIDENCE 0.35f
#define MAX30102_MIN_CONFIDENCE 0.30f
#endif

/* AC signal amplitude quality range (RMS units in ADC counts). */
#ifndef MAX30102_SIGNAL_AC_MIN
#define MAX30102_SIGNAL_AC_MIN 40.0f
#endif

#ifndef MAX30102_SIGNAL_AC_MAX
#define MAX30102_SIGNAL_AC_MAX 25000.0f
#endif

/*
 * FIFO sample averaging options encoded for FIFO_CONFIG[7:5].
 * Higher averaging reduces noise but also smooths sharp waveform detail.
 */
typedef enum {
    MAX30102_SAMPLE_AVG_1 = 0,
    MAX30102_SAMPLE_AVG_2 = 1,
    MAX30102_SAMPLE_AVG_4 = 2,
    MAX30102_SAMPLE_AVG_8 = 3,
    MAX30102_SAMPLE_AVG_16 = 4,
    MAX30102_SAMPLE_AVG_32 = 5,
} max30102_sample_average_t;

/*
 * MODE_CONFIG mode values.
 *
 * HR mode = single LED mode.
 * SPO2 mode = RED + IR mode (required for SpO2 ratio path).
 */
typedef enum {
    MAX30102_MODE_HR = 0x02,
    MAX30102_MODE_SPO2 = 0x03,
} max30102_mode_t;

/* SPO2_CONFIG[6:5] ADC full-scale range options. */
typedef enum {
    MAX30102_ADC_RANGE_2048 = 0,
    MAX30102_ADC_RANGE_4096 = 1,
    MAX30102_ADC_RANGE_8192 = 2,
    MAX30102_ADC_RANGE_16384 = 3,
} max30102_adc_range_t;

/* SPO2_CONFIG[4:2] sample-rate options. */
typedef enum {
    MAX30102_SPO2_SR_50 = 0,
    MAX30102_SPO2_SR_100 = 1,
    MAX30102_SPO2_SR_200 = 2,
    MAX30102_SPO2_SR_400 = 3,
    MAX30102_SPO2_SR_800 = 4,
    MAX30102_SPO2_SR_1000 = 5,
    MAX30102_SPO2_SR_1600 = 6,
    MAX30102_SPO2_SR_3200 = 7,
} max30102_spo2_sample_rate_t;

/* SPO2_CONFIG[1:0] LED pulse width options. */
typedef enum {
    MAX30102_PW_69US = 0,
    MAX30102_PW_118US = 1,
    MAX30102_PW_215US = 2,
    MAX30102_PW_411US = 3,
} max30102_led_pulse_width_t;

/*
 * Runtime configuration used by max30102_init().
 *
 * Field notes:
 * - address: 7-bit I2C address (MAX30102 default is 0x57).
 * - timeout_ms: per-I2C transfer timeout.
 * - sample_average/fifo_rollover_enable/fifo_almost_full: FIFO behavior.
 * - mode/adc_range/sample_rate/pulse_width: SPO2 data path configuration.
 * - led_*_pa: LED pulse amplitudes (register values, not milliamps).
 * - interrupt_enable_1/2: direct values written to interrupt enable regs.
 */
typedef struct {
    uint8_t address;
    uint32_t timeout_ms;
    max30102_sample_average_t sample_average;
    bool fifo_rollover_enable;
    uint8_t fifo_almost_full;
    max30102_mode_t mode;
    max30102_adc_range_t adc_range;
    max30102_spo2_sample_rate_t sample_rate;
    max30102_led_pulse_width_t pulse_width;
    uint8_t led_red_pa;
    uint8_t led_ir_pa;
    uint8_t led_pilot_pa;
    uint8_t interrupt_enable_1;
    uint8_t interrupt_enable_2;
} max30102_config_t;

/*
 * One raw FIFO sample pair.
 *
 * - red/ir are unpacked 18-bit ADC values stored in uint32_t.
 * - These are raw optical ADC counts, not HR or SpO2 values.
 * - timestamp_ms uses esp_log_timestamp() capture time on the host MCU.
 */
typedef struct {
    uint32_t red;
    uint32_t ir;
    uint32_t timestamp_ms;
} max30102_sample_t;

/*
 * Contact quality state reported by algorithm.
 *
 * no_finger   : no reliable optical contact.
 * weak        : contact exists but quality is below valid threshold.
 * valid       : contact is stable enough for estimation attempt.
 */
typedef enum {
    MAX30102_CONTACT_NO_FINGER = 0,
    MAX30102_CONTACT_WEAK,
    MAX30102_CONTACT_VALID,
} max30102_contact_quality_t;

/*
 * Reason code for why HR/SpO2 is currently invalid (shown as NA in app logs).
 *
 * These reason values are useful for debugging why values are rejected.
 */
typedef enum {
    MAX30102_REASON_NONE = 0,
    MAX30102_REASON_NO_CONTACT,
    MAX30102_REASON_WEAK_CONTACT,
    MAX30102_REASON_INSUFFICIENT_SAMPLES,
    MAX30102_REASON_SIGNAL_WEAK,
    MAX30102_REASON_SIGNAL_SATURATED,
    MAX30102_REASON_SIGNAL_NOISY,
    MAX30102_REASON_UNSTABLE_BEATS,
    MAX30102_REASON_INVALID_RATIO,
} max30102_reason_t;

/*
 * Algorithm output snapshot from max30102_algo_update().
 *
 * This struct keeps both raw-ish and processed values so app layer can:
 * - print diagnostics,
 * - send telemetry,
 * - apply policy (NA vs valid output) with context.
 */
typedef struct {
    uint32_t red_raw;
    uint32_t ir_raw;
    float red_filtered;
    float ir_filtered;
    float red_dc;
    float ir_dc;
    float red_ac_rms;
    float ir_ac_rms;
    float heart_rate_bpm;
    float spo2_percent;
    float heart_rate_confidence;
    float spo2_confidence;
    float raw_heart_rate_bpm;
    float raw_spo2_percent;
    float ratio;
    float peak_threshold;
    float motion_metric;
    uint16_t beat_interval_ms;
    bool beat_detected;
    max30102_contact_quality_t contact_quality;
    max30102_reason_t hr_reason;
    max30102_reason_t spo2_reason;
    bool signal_saturated;
    bool heart_rate_valid;
    bool spo2_valid;
} max30102_processed_t;

/*
 * Internal algorithm state (persistent across samples).
 *
 * Why this state exists:
 * - PPG analysis needs history (filters, peaks, beat intervals).
 * - A single sample is not enough to compute stable HR/SpO2.
 */
typedef struct {
    float sample_rate_hz;

    /* DC baseline trackers (slow-moving baseline for each channel). */
    float red_dc;
    float ir_dc;

    /* AC filtered components (pulse waveform after DC subtraction). */
    float red_ac_filtered;
    float ir_ac_filtered;
    /* Slow baseline of filtered IR-AC used to remove residual drift before beat detection. */
    float ir_ac_baseline;

    /* Previous AC values for local-peak detection. */
    float prev_ir_ac;
    float prev_prev_ir_ac;

    uint32_t sample_count;
    int32_t last_peak_index;

    /* Rolling AC windows for RMS and ratio quality metrics. */
    float red_ac_window[MAX30102_BUFFER_SIZE];
    float ir_ac_window[MAX30102_BUFFER_SIZE];
    uint16_t ac_window_count;
    uint16_t ac_window_head;
    float red_ac2_sum;
    float ir_ac2_sum;

    /* Beat interval history for HR estimate stability checks. */
    uint16_t beat_intervals[MAX30102_BEAT_HISTORY_SIZE];
    uint8_t beat_count;
    uint8_t beat_head;
    /*
     * Beat polarity lock:
     *  0 = unknown, +1 = maxima, -1 = minima.
     * Locks detector to one extremum type to avoid half-cycle double counting.
     */
    int8_t beat_polarity;
    /* Candidate/reject counters for debugging peak validation bottlenecks. */
    uint32_t peak_candidate_count;
    uint32_t peak_reject_timing_count;
    uint32_t peak_reject_shape_count;
    uint32_t peak_reject_noise_count;
    uint32_t peak_reject_polarity_count;

    /* Motion/noise proxy from derivative of filtered IR AC. */
    float ir_deriv_abs_ema;

    /* Index of last accepted beat used to expire stale BPM when no fresh beats arrive. */
    int32_t last_accepted_peak_index;
    uint16_t last_peak_interval_samples;
    float raw_heart_rate_bpm;
    float raw_spo2_percent;
    float last_ratio;
    float last_peak_threshold;
    float motion_metric;

    /* Hold counters to avoid immediate value dropouts on brief glitches. */
    uint16_t hr_hold_count;
    uint16_t spo2_hold_count;

    uint32_t peak_count;
    uint32_t last_debug_log_ms;

    /* Published outputs and confidences. */
    float heart_rate_bpm;
    float spo2_percent;
    float heart_rate_confidence;
    float spo2_confidence;

    max30102_contact_quality_t contact_quality;
    max30102_reason_t hr_reason;
    max30102_reason_t spo2_reason;

    bool signal_saturated;
    bool heart_rate_valid;
    bool spo2_valid;

    /* Contact transition / debounce / settling control. */
    uint32_t settling_until_ms;
    uint32_t contact_transition_ms;
    uint32_t valid_candidate_start_ms;
    uint16_t valid_bad_sample_streak;
} max30102_algo_state_t;

/*
 * Device instance object.
 *
 * - i2c_dev is passed into i2c_manager helpers.
 * - sample_rate_hz mirrors configured sample rate for algorithm setup.
 * - initialized gate prevents accidental reads before init.
 */
typedef struct {
    i2c_manager_device_t i2c_dev;
    uint32_t timeout_ms;
    float sample_rate_hz;
    bool initialized;
} max30102_t;

/*
 * Initialize MAX30102 device and write all selected configuration registers.
 *
 * Parameters:
 * - device: output device handle (must be non-NULL).
 * - config: optional config; if NULL, internal defaults are used.
 *
 * Returns:
 * - ESP_OK on success.
 * - ESP_ERR_INVALID_ARG for invalid input.
 * - I2C/probe related errors for hardware communication failures.
 */
esp_err_t max30102_init(max30102_t *device, const max30102_config_t *config);

/*
 * Remove MAX30102 device from I2C manager and mark handle uninitialized.
 */
esp_err_t max30102_deinit(max30102_t *device);

/*
 * Issue hardware reset and wait until reset bit clears.
 */
esp_err_t max30102_reset(max30102_t *device);

/*
 * Write interrupt enable registers directly.
 */
esp_err_t max30102_config_interrupts(max30102_t *device, uint8_t int_enable_1, uint8_t int_enable_2);

/*
 * Read interrupt status registers.
 * Note: status reads may clear latched interrupt flags depending on sensor behavior.
 */
esp_err_t max30102_read_interrupt_status(max30102_t *device, uint8_t *int_status_1, uint8_t *int_status_2);

/*
 * Flush FIFO by resetting write/read pointers and overflow counter.
 */
esp_err_t max30102_fifo_flush(max30102_t *device);

/*
 * Read one FIFO sample (RED+IR).
 *
 * Returns:
 * - ESP_OK when a sample is read.
 * - ESP_ERR_NOT_FOUND when FIFO has no new sample.
 * - other ESP_ERR_* values for bus/device errors.
 */
esp_err_t max30102_fifo_read_sample(max30102_t *device, max30102_sample_t *sample);

/*
 * Read internal die temperature in Celsius.
 *
 * Useful as a diagnostics helper, not a replacement for body/ambient sensors.
 */
esp_err_t max30102_read_die_temperature_c(max30102_t *device, float *temperature_c);

/*
 * Initialize algorithm state for a new session.
 */
void max30102_algo_init(max30102_algo_state_t *state, float sample_rate_hz);

/*
 * Process one sample and update HR/SpO2 state.
 *
 * Parameters:
 * - state: algorithm persistent state.
 * - sample: input raw sample.
 * - out: optional output snapshot (can be NULL).
 */
void max30102_algo_update(max30102_algo_state_t *state, const max30102_sample_t *sample, max30102_processed_t *out);

/*
 * Convert enums to stable text for logs/UI.
 */
const char *max30102_contact_quality_to_string(max30102_contact_quality_t quality);
const char *max30102_reason_to_string(max30102_reason_t reason);

/*
 * Helper conversions/accessors.
 */
float max30102_sample_rate_enum_to_hz(max30102_spo2_sample_rate_t sample_rate);
float max30102_get_sample_rate_hz(const max30102_t *device);

#endif
