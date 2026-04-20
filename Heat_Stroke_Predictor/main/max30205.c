#include "max30205.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"

/*
 * MAX30205 register map used by this driver.
 *
 * 0x00: Temperature register (MSB/LSB, signed, 1/256 C per LSB)
 * 0x01: Configuration register
 * 0x02: THYST threshold register
 * 0x03: TOS threshold register
 */
#define MAX30205_REG_TEMPERATURE 0x00U
#define MAX30205_REG_CONFIGURATION 0x01U
#define MAX30205_REG_THYST 0x02U
#define MAX30205_REG_TOS 0x03U

/* Default values used when caller passes NULL or zero timeout. */
#define MAX30205_ADDR_DEFAULT 0x48U
#define MAX30205_TIMEOUT_DEFAULT_MS 100U

/* MAX30205 output scale: one raw LSB equals 1/256 degree Celsius. */
#define MAX30205_TEMPERATURE_RESOLUTION_C (1.0f / 256.0f)

static const char *TAG = "max30205";

/*
 * Internal readiness check.
 *
 * Why this helper exists:
 * - Avoid duplicate NULL + initialized checks in every public API.
 * - Keep read path defensive against using an uninitialized handle.
 */
static bool max30205_is_ready(const max30205_t *device)
{
    return (device != NULL) && device->initialized;
}

/*
 * Convert Celsius float to MAX30205 signed raw format.
 *
 * We clamp to sensor representable range so conversion is well-defined.
 * Then we round to nearest integer LSB to minimize conversion bias.
 */
static int16_t max30205_temperature_to_raw(float temperature_c)
{
    float limited = temperature_c;
    if (limited > 127.996f) {
        limited = 127.996f;
    } else if (limited < -128.0f) {
        limited = -128.0f;
    }

    return (int16_t)lroundf(limited / MAX30205_TEMPERATURE_RESOLUTION_C);
}

/* Convert signed raw register value back into Celsius. */
static float max30205_raw_to_temperature(int16_t raw_temp)
{
    return (float)raw_temp * MAX30205_TEMPERATURE_RESOLUTION_C;
}

/*
 * Helper to write THYST/TOS style temperature registers.
 *
 * Register wire format is big-endian: MSB first, then LSB.
 */
static esp_err_t max30205_write_temperature_reg(max30205_t *device, uint8_t reg, float value_c)
{
    const int16_t raw = max30205_temperature_to_raw(value_c);
    uint8_t payload[2];
    payload[0] = (uint8_t)((raw >> 8) & 0xFFU); /* High byte */
    payload[1] = (uint8_t)(raw & 0xFFU);         /* Low byte */
    return i2c_manager_write_reg(&device->i2c_dev, reg, payload, sizeof(payload), device->timeout_ms);
}

/*
 * Initialize MAX30205:
 * 1) Apply defaults + caller config
 * 2) Probe bus address
 * 3) Register device with shared I2C manager
 * 4) Write configuration and optional thresholds
 */
esp_err_t max30205_init(max30205_t *device, const max30205_device_config_t *config)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    max30205_device_config_t cfg = {
        .address = MAX30205_ADDR_DEFAULT,
        .timeout_ms = MAX30205_TIMEOUT_DEFAULT_MS,
        .config_reg = 0,
        .low_threshold_c = 35.0f,
        .high_threshold_c = 38.0f,
        .apply_thresholds = false,
    };

    if (config != NULL) {
        cfg = *config;
    }

    /* Start from a known-zero state to avoid stale handles from previous runs. */
    memset(device, 0, sizeof(*device));
    device->timeout_ms = (cfg.timeout_ms == 0U) ? MAX30205_TIMEOUT_DEFAULT_MS : cfg.timeout_ms;
    device->i2c_dev.address = cfg.address;

    /*
     * Keep bus speed explicit for this sensor handle.
     * MAX30205 works with lower speeds too, but 400 kHz is typically fine.
     */
    device->i2c_dev.scl_speed_hz = 400000U;

    /* Quick ACK probe first so failures are reported early and clearly. */
    esp_err_t err = i2c_manager_probe(cfg.address, device->timeout_ms);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MAX30205 probe failed at 0x%02X (%s)", cfg.address, esp_err_to_name(err));
        return err;
    }

    err = i2c_manager_add_device(&device->i2c_dev);
    if (err != ESP_OK) {
        return err;
    }

    err = max30205_set_config(device, cfg.config_reg);
    if (err != ESP_OK) {
        max30205_deinit(device);
        return err;
    }

    if (cfg.apply_thresholds) {
        err = max30205_set_thresholds(device, cfg.low_threshold_c, cfg.high_threshold_c);
        if (err != ESP_OK) {
            max30205_deinit(device);
            return err;
        }
    }

    device->initialized = true;
    ESP_LOGI(TAG, "MAX30205 initialized at 0x%02X", cfg.address);
    return ESP_OK;
}

/*
 * Remove device from I2C manager and clear ready flag.
 *
 * Note: remove helper is called even if already deinitialized; caller receives
 * i2c_manager_remove_device() status so failures remain visible.
 */
esp_err_t max30205_deinit(max30205_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    device->initialized = false;
    return i2c_manager_remove_device(&device->i2c_dev);
}

/* Write complete configuration register byte. */
esp_err_t max30205_set_config(max30205_t *device, uint8_t config_reg)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_manager_write_reg_u8(&device->i2c_dev, MAX30205_REG_CONFIGURATION, config_reg, device->timeout_ms);
}

/* Read complete configuration register byte. */
esp_err_t max30205_get_config(max30205_t *device, uint8_t *config_reg)
{
    if ((device == NULL) || (config_reg == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_manager_read_reg_u8(&device->i2c_dev, MAX30205_REG_CONFIGURATION, config_reg, device->timeout_ms);
}

/*
 * Toggle shutdown bit without changing other configuration bits.
 *
 * Read-modify-write is used so interrupt/fault settings are preserved.
 */
esp_err_t max30205_set_shutdown(max30205_t *device, bool enable)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t config_reg = 0;
    esp_err_t err = max30205_get_config(device, &config_reg);
    if (err != ESP_OK) {
        return err;
    }

    if (enable) {
        config_reg |= MAX30205_CONFIG_SHUTDOWN;
    } else {
        config_reg &= (uint8_t)(~MAX30205_CONFIG_SHUTDOWN);
    }

    return max30205_set_config(device, config_reg);
}

/*
 * Trigger one-shot conversion.
 *
 * This bit is meaningful in shutdown mode where periodic conversions are not running.
 */
esp_err_t max30205_trigger_one_shot(max30205_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t config_reg = 0;
    esp_err_t err = max30205_get_config(device, &config_reg);
    if (err != ESP_OK) {
        return err;
    }

    config_reg |= MAX30205_CONFIG_ONE_SHOT;
    return max30205_set_config(device, config_reg);
}

/*
 * Program low/high threshold registers.
 *
 * Guarding low <= high avoids invalid threshold ordering.
 */
esp_err_t max30205_set_thresholds(max30205_t *device, float low_threshold_c, float high_threshold_c)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (low_threshold_c > high_threshold_c) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = max30205_write_temperature_reg(device, MAX30205_REG_THYST, low_threshold_c);
    if (err != ESP_OK) {
        return err;
    }
    return max30205_write_temperature_reg(device, MAX30205_REG_TOS, high_threshold_c);
}

/*
 * Read latest temperature sample in Celsius.
 *
 * Byte assembly details:
 * - raw[0] = MSB
 * - raw[1] = LSB
 * - combined value is signed 16-bit two's complement.
 */
esp_err_t max30205_read_temperature_c(max30205_t *device, float *temperature_c)
{
    if (!max30205_is_ready(device) || (temperature_c == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[2];
    esp_err_t err = i2c_manager_read_reg(&device->i2c_dev, MAX30205_REG_TEMPERATURE, raw, sizeof(raw), device->timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    const int16_t temp_raw = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    *temperature_c = max30205_raw_to_temperature(temp_raw);
    return ESP_OK;
}
