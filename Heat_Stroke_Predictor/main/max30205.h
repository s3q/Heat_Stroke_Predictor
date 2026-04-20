#ifndef MAX30205_H
#define MAX30205_H

/*
 * MAX30205 driver interface (ESP-IDF + I2C manager)
 *
 * What this driver does:
 * - Initializes a MAX30205 temperature sensor on the shared I2C bus.
 * - Reads temperature in degree Celsius.
 * - Allows optional configuration of comparator/interrupt behavior.
 * - Allows writing low/high threshold registers (THYST/TOS).
 *
 * Typical usage order:
 * 1) Fill max30205_device_config_t (or pass NULL for defaults)
 * 2) max30205_init(...)
 * 3) Periodically call max30205_read_temperature_c(...)
 * 4) Optional: max30205_set_shutdown(...) for low-power mode
 * 5) max30205_deinit(...) on shutdown/re-init
 *
 * Important:
 * - This driver reports the sensor temperature value only.
 * - Any "estimated body temperature" logic belongs in application code
 *   (for example, filtering/calibration/contact checks in app_logic.c).
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "i2c_manager.h"

/*
 * MAX30205 configuration-register bit helpers.
 *
 * These values map directly to the MAX30205 CONFIG register bits.
 * They are exposed so application code can build a config byte clearly.
 */
#define MAX30205_CONFIG_SHUTDOWN      (1U << 0)
#define MAX30205_CONFIG_INTERRUPT     (1U << 1)
#define MAX30205_CONFIG_OS_COMP_INT   (1U << 2)
#define MAX30205_CONFIG_FAULT_QUEUE_1 (0U << 3)
#define MAX30205_CONFIG_FAULT_QUEUE_2 (1U << 3)
#define MAX30205_CONFIG_FAULT_QUEUE_4 (2U << 3)
#define MAX30205_CONFIG_FAULT_QUEUE_6 (3U << 3)
#define MAX30205_CONFIG_TIMEOUT       (1U << 5)
#define MAX30205_CONFIG_OS_POLARITY   (1U << 6)
#define MAX30205_CONFIG_ONE_SHOT      (1U << 7)

/*
 * Startup configuration passed into max30205_init().
 *
 * Field notes:
 * - address: 7-bit I2C address (MAX30205 is commonly 0x48).
 * - timeout_ms: I2C transfer timeout used by i2c_manager helpers.
 * - config_reg: full CONFIG register value written during init.
 * - low_threshold_c/high_threshold_c: optional THYST/TOS values.
 * - apply_thresholds: when true, THYST/TOS are written during init.
 */
typedef struct {
    uint8_t address;
    uint32_t timeout_ms;
    uint8_t config_reg;
    float low_threshold_c;
    float high_threshold_c;
    bool apply_thresholds;
} max30205_device_config_t;

/*
 * Runtime device object.
 *
 * - i2c_dev: handle used by i2c_manager API calls.
 * - timeout_ms: cached transfer timeout for all register operations.
 * - initialized: safety gate used by read functions.
 */
typedef struct {
    i2c_manager_device_t i2c_dev;
    uint32_t timeout_ms;
    bool initialized;
} max30205_t;

/*
 * Initialize sensor and attach it to I2C manager.
 *
 * Returns:
 * - ESP_OK on success.
 * - ESP_ERR_INVALID_ARG for invalid inputs.
 * - I2C-related errors if probe or register writes fail.
 */
esp_err_t max30205_init(max30205_t *device, const max30205_device_config_t *config);

/*
 * Deinitialize device and remove it from I2C manager.
 */
esp_err_t max30205_deinit(max30205_t *device);

/*
 * Write/read full CONFIG register value.
 */
esp_err_t max30205_set_config(max30205_t *device, uint8_t config_reg);
esp_err_t max30205_get_config(max30205_t *device, uint8_t *config_reg);

/*
 * Enable/disable shutdown mode by updating CONFIG[0].
 * In shutdown mode, conversion activity is reduced to save power.
 */
esp_err_t max30205_set_shutdown(max30205_t *device, bool enable);

/*
 * Trigger one-shot conversion by setting CONFIG[7].
 * Commonly used when sensor is in shutdown mode.
 */
esp_err_t max30205_trigger_one_shot(max30205_t *device);

/*
 * Write hysteresis and over-temperature thresholds.
 *
 * - low_threshold_c  -> THYST register
 * - high_threshold_c -> TOS register
 */
esp_err_t max30205_set_thresholds(max30205_t *device, float low_threshold_c, float high_threshold_c);

/*
 * Read current temperature in degree Celsius.
 *
 * Returns:
 * - ESP_OK and writes output when read succeeds.
 * - ESP_ERR_INVALID_ARG for bad handle/output pointer.
 * - I2C errors for transfer failures.
 */
esp_err_t max30205_read_temperature_c(max30205_t *device, float *temperature_c);

#endif
