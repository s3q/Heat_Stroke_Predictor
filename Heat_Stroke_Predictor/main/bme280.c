#include "bme280.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BME280_REG_CALIB00 0x88U
#define BME280_REG_CALIB26 0xE1U
#define BME280_REG_H1 0xA1U
#define BME280_REG_CHIP_ID 0xD0U
#define BME280_REG_RESET 0xE0U
#define BME280_REG_CTRL_HUM 0xF2U
#define BME280_REG_STATUS 0xF3U
#define BME280_REG_CTRL_MEAS 0xF4U
#define BME280_REG_CONFIG 0xF5U
#define BME280_REG_PRESS_MSB 0xF7U

#define BME280_CHIP_ID_BME280 0x60U
#define BME280_CHIP_ID_BMP280 0x58U
#define BME280_SOFT_RESET_CMD 0xB6U

#define BME280_STATUS_MEASURING_MASK (1U << 3)
#define BME280_STATUS_IM_UPDATE_MASK (1U << 0)
#define BME280_TIMEOUT_DEFAULT_MS 100U
#define BME280_ADDRESS_DEFAULT 0x76U

#ifndef BME280_ENABLE_VERBOSE_LOG
#define BME280_ENABLE_VERBOSE_LOG 0
#endif

#ifndef BME280_ENABLE_CALIB_LOG
#define BME280_ENABLE_CALIB_LOG 0
#endif

static const char *TAG = "bme280";

#if BME280_ENABLE_VERBOSE_LOG
#define BME280_VLOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define BME280_VLOGI(...) ((void)0)
#endif

static bool bme280_is_ready(const bme280_t *device)
{
    return (device != NULL) && device->initialized;
}

static uint16_t bme280_u16_le(const uint8_t *buf)
{
    return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

static int16_t bme280_s16_le(const uint8_t *buf)
{
    return (int16_t)bme280_u16_le(buf);
}

static int16_t bme280_sign_extend_12(uint16_t value)
{
    if ((value & 0x800U) != 0U) {
        value |= 0xF000U;
    }
    return (int16_t)value;
}

static esp_err_t bme280_write_reg_u8(bme280_t *device, uint8_t reg, uint8_t value)
{
    return i2c_manager_write_reg_u8(&device->i2c_dev, reg, value, device->timeout_ms);
}

static esp_err_t bme280_read_reg_u8(bme280_t *device, uint8_t reg, uint8_t *value)
{
    return i2c_manager_read_reg_u8(&device->i2c_dev, reg, value, device->timeout_ms);
}

static void bme280_log_probe_result(uint8_t address, esp_err_t err)
{
    if (err == ESP_OK) {
        BME280_VLOGI("BME280 low-level probe ACK at 0x%02X", address);
        return;
    }

    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "BME280 no ACK at 0x%02X", address);
        return;
    }

    ESP_LOGE(TAG, "BME280 I2C probe error at 0x%02X: %s", address, esp_err_to_name(err));
}

static esp_err_t bme280_check_chip_id(uint8_t chip_id)
{
    if (chip_id == BME280_CHIP_ID_BME280) {
        return ESP_OK;
    }

    if (chip_id == BME280_CHIP_ID_BMP280) {
        ESP_LOGW(TAG, "CHIP_ID=0x%02X indicates BMP280, not BME280", chip_id);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGW(TAG, "Wrong chip ID: 0x%02X (expected BME280 0x%02X)", chip_id, BME280_CHIP_ID_BME280);
    return ESP_ERR_INVALID_RESPONSE;
}

static void bme280_dump_calibration(const bme280_t *device)
{
#if BME280_ENABLE_CALIB_LOG
    ESP_LOGI(TAG,
             "BME280 calib T: T1=%u T2=%d T3=%d",
             (unsigned)device->calib.dig_t1,
             (int)device->calib.dig_t2,
             (int)device->calib.dig_t3);
    ESP_LOGI(TAG,
             "BME280 calib P: P1=%u P2=%d P3=%d P4=%d P5=%d P6=%d P7=%d P8=%d P9=%d",
             (unsigned)device->calib.dig_p1,
             (int)device->calib.dig_p2,
             (int)device->calib.dig_p3,
             (int)device->calib.dig_p4,
             (int)device->calib.dig_p5,
             (int)device->calib.dig_p6,
             (int)device->calib.dig_p7,
             (int)device->calib.dig_p8,
             (int)device->calib.dig_p9);
    ESP_LOGI(TAG,
             "BME280 calib H: H1=%u H2=%d H3=%u H4=%d H5=%d H6=%d",
             (unsigned)device->calib.dig_h1,
             (int)device->calib.dig_h2,
             (unsigned)device->calib.dig_h3,
             (int)device->calib.dig_h4,
             (int)device->calib.dig_h5,
             (int)device->calib.dig_h6);
#else
    (void)device;
#endif
}

static bool bme280_calibration_is_sane(const bme280_t *device)
{
    if ((device->calib.dig_t1 == 0U) || (device->calib.dig_t1 == 0xFFFFU)) {
        return false;
    }
    if ((device->calib.dig_p1 == 0U) || (device->calib.dig_p1 == 0xFFFFU)) {
        return false;
    }
    return true;
}

static void bme280_sanitize_config(bme280_config_t *cfg)
{
    if (cfg->osrs_t == BME280_OSRS_SKIPPED) {
        ESP_LOGW(TAG, "BME280 temperature oversampling was SKIPPED; overriding to X1");
        cfg->osrs_t = BME280_OSRS_X1;
    }
    if (cfg->osrs_p == BME280_OSRS_SKIPPED) {
        ESP_LOGW(TAG, "BME280 pressure oversampling was SKIPPED; overriding to X1");
        cfg->osrs_p = BME280_OSRS_X1;
    }
}

static esp_err_t bme280_wait_nvm_copy_done(bme280_t *device, uint32_t timeout_ms)
{
    const uint32_t poll_ms = 2U;
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms <= timeout_ms) {
        uint8_t status = 0U;
        esp_err_t err = bme280_read_reg_u8(device, BME280_REG_STATUS, &status);
        if (err != ESP_OK) {
            return err;
        }

        if ((status & BME280_STATUS_IM_UPDATE_MASK) == 0U) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        elapsed_ms += poll_ms;
    }

    return ESP_ERR_TIMEOUT;
}

static bme280_config_t bme280_default_config(void)
{
    const bme280_config_t cfg = {
        .address = BME280_ADDRESS_DEFAULT,
        .timeout_ms = BME280_TIMEOUT_DEFAULT_MS,
        .osrs_t = BME280_OSRS_X2,
        .osrs_p = BME280_OSRS_X16,
        .osrs_h = BME280_OSRS_X1,
        .mode = BME280_MODE_NORMAL,
        .standby_time = BME280_STANDBY_500_MS,
        .filter = BME280_FILTER_4,
    };
    return cfg;
}

static esp_err_t bme280_read_calibration_data(bme280_t *device)
{
    uint8_t calib_00_25[26];
    esp_err_t err =
        i2c_manager_read_reg(&device->i2c_dev, BME280_REG_CALIB00, calib_00_25, sizeof(calib_00_25), device->timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    device->calib.dig_t1 = bme280_u16_le(&calib_00_25[0]);
    device->calib.dig_t2 = bme280_s16_le(&calib_00_25[2]);
    device->calib.dig_t3 = bme280_s16_le(&calib_00_25[4]);
    device->calib.dig_p1 = bme280_u16_le(&calib_00_25[6]);
    device->calib.dig_p2 = bme280_s16_le(&calib_00_25[8]);
    device->calib.dig_p3 = bme280_s16_le(&calib_00_25[10]);
    device->calib.dig_p4 = bme280_s16_le(&calib_00_25[12]);
    device->calib.dig_p5 = bme280_s16_le(&calib_00_25[14]);
    device->calib.dig_p6 = bme280_s16_le(&calib_00_25[16]);
    device->calib.dig_p7 = bme280_s16_le(&calib_00_25[18]);
    device->calib.dig_p8 = bme280_s16_le(&calib_00_25[20]);
    device->calib.dig_p9 = bme280_s16_le(&calib_00_25[22]);

    err = bme280_read_reg_u8(device, BME280_REG_H1, &device->calib.dig_h1);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t calib_26_32[7];
    err = i2c_manager_read_reg(&device->i2c_dev, BME280_REG_CALIB26, calib_26_32, sizeof(calib_26_32), device->timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    device->calib.dig_h2 = bme280_s16_le(&calib_26_32[0]);
    device->calib.dig_h3 = calib_26_32[2];

    const uint16_t h4_raw = (uint16_t)(((uint16_t)calib_26_32[3] << 4) | (calib_26_32[4] & 0x0FU));
    const uint16_t h5_raw = (uint16_t)(((uint16_t)calib_26_32[5] << 4) | (calib_26_32[4] >> 4));
    device->calib.dig_h4 = bme280_sign_extend_12(h4_raw);
    device->calib.dig_h5 = bme280_sign_extend_12(h5_raw);
    device->calib.dig_h6 = (int8_t)calib_26_32[6];

    return ESP_OK;
}

static esp_err_t bme280_apply_settings(bme280_t *device, const bme280_config_t *config)
{
    const uint8_t ctrl_hum = (uint8_t)(config->osrs_h & 0x07U);
    const uint8_t reg_config =
        (uint8_t)(((uint8_t)(config->standby_time & 0x07U) << 5) | ((uint8_t)(config->filter & 0x07U) << 2));
    const uint8_t ctrl_meas = (uint8_t)(((uint8_t)(config->osrs_t & 0x07U) << 5) |
                                        ((uint8_t)(config->osrs_p & 0x07U) << 2) | ((uint8_t)(config->mode & 0x03U)));

    esp_err_t err = bme280_write_reg_u8(device, BME280_REG_CTRL_HUM, ctrl_hum);
    if (err != ESP_OK) {
        return err;
    }

    err = bme280_write_reg_u8(device, BME280_REG_CONFIG, reg_config);
    if (err != ESP_OK) {
        return err;
    }

    err = bme280_write_reg_u8(device, BME280_REG_CTRL_MEAS, ctrl_meas);
    if (err != ESP_OK) {
        return err;
    }

    device->mode = config->mode;
    return ESP_OK;
}

static esp_err_t bme280_wait_measurement_done(bme280_t *device, uint32_t timeout_ms)
{
    const uint32_t poll_ms = 5U;
    uint32_t elapsed = 0U;

    while (elapsed <= timeout_ms) {
        uint8_t status = 0;
        esp_err_t err = bme280_read_reg_u8(device, BME280_REG_STATUS, &status);
        if (err != ESP_OK) {
            return err;
        }

        if ((status & BME280_STATUS_MEASURING_MASK) == 0U) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        elapsed += poll_ms;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t bme280_init(bme280_t *device, const bme280_config_t *config)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bme280_config_t cfg = bme280_default_config();
    if (config != NULL) {
        cfg = *config;
    }
    bme280_sanitize_config(&cfg);

    memset(device, 0, sizeof(*device));
    device->timeout_ms = (cfg.timeout_ms == 0U) ? BME280_TIMEOUT_DEFAULT_MS : cfg.timeout_ms;
    device->i2c_dev.address = cfg.address;
    device->i2c_dev.scl_speed_hz = 0U;

    BME280_VLOGI("BME280 low-level check at 0x%02X: probing + CHIP_ID read", cfg.address);
    esp_err_t err = i2c_manager_probe(cfg.address, device->timeout_ms);
    bme280_log_probe_result(cfg.address, err);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_manager_add_device(&device->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME280 I2C device add failed at 0x%02X: %s", cfg.address, esp_err_to_name(err));
        return err;
    }

    uint8_t chip_id = 0;
    err = bme280_read_reg_u8(device, BME280_REG_CHIP_ID, &chip_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME280 CHIP_ID read error at 0x%02X: %s", cfg.address, esp_err_to_name(err));
        bme280_deinit(device);
        return err;
    }

    BME280_VLOGI("BME280 CHIP_ID register 0x%02X = 0x%02X", BME280_REG_CHIP_ID, chip_id);
    err = bme280_check_chip_id(chip_id);
    if (err != ESP_OK) {
        bme280_deinit(device);
        return err;
    }

    err = bme280_write_reg_u8(device, BME280_REG_RESET, BME280_SOFT_RESET_CMD);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME280 soft-reset write failed: %s", esp_err_to_name(err));
        bme280_deinit(device);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(3));

    err = bme280_wait_nvm_copy_done(device, 30U);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME280 NVM copy wait failed: %s", esp_err_to_name(err));
        bme280_deinit(device);
        return err;
    }

    err = bme280_read_calibration_data(device);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME280 calibration data read failure: %s", esp_err_to_name(err));
        bme280_deinit(device);
        return err;
    }
    bme280_dump_calibration(device);
    if (!bme280_calibration_is_sane(device)) {
        ESP_LOGE(TAG, "BME280 calibration sanity check failed (T1/P1 invalid)");
        bme280_deinit(device);
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = bme280_apply_settings(device, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME280 configuration write/readback failure: %s", esp_err_to_name(err));
        bme280_deinit(device);
        return err;
    }

    device->initialized = true;
    ESP_LOGI(TAG, "BME280 initialized at 0x%02X", cfg.address);
    return ESP_OK;
}

esp_err_t bme280_deinit(bme280_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    device->initialized = false;
    return i2c_manager_remove_device(&device->i2c_dev);
}

esp_err_t bme280_set_mode(bme280_t *device, bme280_mode_t mode)
{
    if (!bme280_is_ready(device)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t ctrl_meas = 0;
    esp_err_t err = bme280_read_reg_u8(device, BME280_REG_CTRL_MEAS, &ctrl_meas);
    if (err != ESP_OK) {
        return err;
    }

    ctrl_meas = (uint8_t)((ctrl_meas & 0xFCU) | ((uint8_t)mode & 0x03U));
    err = bme280_write_reg_u8(device, BME280_REG_CTRL_MEAS, ctrl_meas);
    if (err != ESP_OK) {
        return err;
    }

    device->mode = mode;
    return ESP_OK;
}

esp_err_t bme280_read_measurements(bme280_t *device, bme280_measurements_t *measurements)
{
    if (!bme280_is_ready(device) || (measurements == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t ctrl_meas = 0U;
    esp_err_t err = bme280_read_reg_u8(device, BME280_REG_CTRL_MEAS, &ctrl_meas);
    if (err != ESP_OK) {
        return err;
    }
    const bme280_mode_t current_mode = (bme280_mode_t)(ctrl_meas & 0x03U);

    if (device->mode == BME280_MODE_FORCED) {
        err = bme280_set_mode(device, BME280_MODE_FORCED);
        if (err != ESP_OK) {
            return err;
        }

        err = bme280_wait_measurement_done(device, 200U);
        if (err != ESP_OK) {
            return err;
        }
    } else if ((device->mode == BME280_MODE_NORMAL) && (current_mode != BME280_MODE_NORMAL)) {
        ESP_LOGW(TAG, "BME280 mode drift detected (ctrl_meas mode=%u), restoring NORMAL mode", (unsigned)current_mode);
        err = bme280_set_mode(device, BME280_MODE_NORMAL);
        if (err != ESP_OK) {
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    uint8_t raw[8];
    err = i2c_manager_read_reg(&device->i2c_dev, BME280_REG_PRESS_MSB, raw, sizeof(raw), device->timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    const int32_t adc_p = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | ((uint32_t)raw[2] >> 4));
    const int32_t adc_t = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | ((uint32_t)raw[5] >> 4));
    const int32_t adc_h = (int32_t)(((uint32_t)raw[6] << 8) | raw[7]);

    BME280_VLOGI("BME280 raw ADC: temp=%" PRId32 " press=%" PRId32 " hum=%" PRId32, adc_t, adc_p, adc_h);

    /* Temperature compensation first: computes t_fine for pressure/humidity paths. */
    double var1 = ((double)adc_t / 16384.0) - ((double)device->calib.dig_t1 / 1024.0);
    var1 *= (double)device->calib.dig_t2;
    double var2 = ((double)adc_t / 131072.0) - ((double)device->calib.dig_t1 / 8192.0);
    var2 = var2 * var2 * (double)device->calib.dig_t3;
    const double t_fine_d = var1 + var2;
    device->t_fine = (int32_t)t_fine_d;
    double temperature_c = t_fine_d / 5120.0;

    /* Pressure compensation uses t_fine from temperature compensation. */
    double p_var1 = (t_fine_d / 2.0) - 64000.0;
    double p_var2 = p_var1 * p_var1 * ((double)device->calib.dig_p6 / 32768.0);
    p_var2 += p_var1 * (double)device->calib.dig_p5 * 2.0;
    p_var2 = (p_var2 / 4.0) + ((double)device->calib.dig_p4 * 65536.0);
    p_var1 = (((double)device->calib.dig_p3 * p_var1 * p_var1) / 524288.0 +
              ((double)device->calib.dig_p2 * p_var1)) /
             524288.0;
    p_var1 = (1.0 + (p_var1 / 32768.0)) * (double)device->calib.dig_p1;

    double pressure_pa = 0.0;
    if (fabs(p_var1) > 1e-12) {
        pressure_pa = 1048576.0 - (double)adc_p;
        pressure_pa = ((pressure_pa - (p_var2 / 4096.0)) * 6250.0) / p_var1;
        p_var1 = ((double)device->calib.dig_p9 * pressure_pa * pressure_pa) / 2147483648.0;
        p_var2 = pressure_pa * ((double)device->calib.dig_p8 / 32768.0);
        pressure_pa += (p_var1 + p_var2 + (double)device->calib.dig_p7) / 16.0;
    } else {
        ESP_LOGW(TAG, "BME280 pressure compensation denominator is zero (dig_p1=%u)", (unsigned)device->calib.dig_p1);
    }

    /* Humidity compensation also depends on t_fine. */
    double humidity = t_fine_d - 76800.0;
    humidity = (adc_h - (((double)device->calib.dig_h4 * 64.0) + (((double)device->calib.dig_h5) / 16384.0) * humidity)) *
               (((double)device->calib.dig_h2 / 65536.0) *
                (1.0 + ((double)device->calib.dig_h6 / 67108864.0) * humidity *
                           (1.0 + ((double)device->calib.dig_h3 / 67108864.0) * humidity)));
    humidity = humidity * (1.0 - ((double)device->calib.dig_h1 * humidity / 524288.0));

    if (humidity > 100.0) {
        humidity = 100.0;
    } else if (humidity < 0.0) {
        humidity = 0.0;
    }

    measurements->temperature_c = (float)temperature_c;
    measurements->pressure_hpa = (float)(pressure_pa / 100.0);
    measurements->humidity_percent = (float)humidity;

    BME280_VLOGI("BME280 compensated: temp=%.2f C, press=%.2f hPa, hum=%.2f %%, t_fine=%" PRId32,
                 measurements->temperature_c,
                 measurements->pressure_hpa,
                 measurements->humidity_percent,
                 device->t_fine);
    return ESP_OK;
}
