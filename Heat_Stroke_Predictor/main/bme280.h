#ifndef BME280_H
#define BME280_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "i2c_manager.h"

typedef enum {
    BME280_OSRS_SKIPPED = 0,
    BME280_OSRS_X1 = 1,
    BME280_OSRS_X2 = 2,
    BME280_OSRS_X4 = 3,
    BME280_OSRS_X8 = 4,
    BME280_OSRS_X16 = 5,
} bme280_oversampling_t;

typedef enum {
    BME280_MODE_SLEEP = 0,
    BME280_MODE_FORCED = 1,
    BME280_MODE_NORMAL = 3,
} bme280_mode_t;

typedef enum {
    BME280_STANDBY_0P5_MS = 0,
    BME280_STANDBY_62P5_MS = 1,
    BME280_STANDBY_125_MS = 2,
    BME280_STANDBY_250_MS = 3,
    BME280_STANDBY_500_MS = 4,
    BME280_STANDBY_1000_MS = 5,
    BME280_STANDBY_10_MS = 6,
    BME280_STANDBY_20_MS = 7,
} bme280_standby_t;

typedef enum {
    BME280_FILTER_OFF = 0,
    BME280_FILTER_2 = 1,
    BME280_FILTER_4 = 2,
    BME280_FILTER_8 = 3,
    BME280_FILTER_16 = 4,
} bme280_filter_t;

typedef struct {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;
} bme280_calibration_data_t;

typedef struct {
    uint8_t address;
    uint32_t timeout_ms;
    bme280_oversampling_t osrs_t;
    bme280_oversampling_t osrs_p;
    bme280_oversampling_t osrs_h;
    bme280_mode_t mode;
    bme280_standby_t standby_time;
    bme280_filter_t filter;
} bme280_config_t;

typedef struct {
    float temperature_c;
    float humidity_percent;
    float pressure_hpa;
} bme280_measurements_t;

typedef struct {
    i2c_manager_device_t i2c_dev;
    bme280_calibration_data_t calib;
    int32_t t_fine;
    uint32_t timeout_ms;
    bme280_mode_t mode;
    bool initialized;
} bme280_t;

esp_err_t bme280_init(bme280_t *device, const bme280_config_t *config);
esp_err_t bme280_deinit(bme280_t *device);
esp_err_t bme280_set_mode(bme280_t *device, bme280_mode_t mode);
esp_err_t bme280_read_measurements(bme280_t *device, bme280_measurements_t *measurements);

#endif
