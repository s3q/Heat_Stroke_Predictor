#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_port_num_t port;
    gpio_num_t sda_io_num;
    gpio_num_t scl_io_num;
    uint32_t clk_speed_hz;
    uint8_t glitch_ignore_cnt;
    bool enable_internal_pullup;
} i2c_manager_config_t;

typedef struct {
    uint8_t address;
    uint32_t scl_speed_hz;
    i2c_master_dev_handle_t handle;
    bool is_added;
} i2c_manager_device_t;

typedef struct {
    uint8_t addresses[0x78U - 0x08U];
    size_t count;
} i2c_manager_scan_result_t;

esp_err_t i2c_manager_init(const i2c_manager_config_t *config);
esp_err_t i2c_manager_deinit(void);
bool i2c_manager_is_initialized(void);

esp_err_t i2c_manager_probe(uint8_t address, uint32_t timeout_ms);
esp_err_t i2c_manager_scan(i2c_manager_scan_result_t *result, uint32_t timeout_ms_per_addr);
esp_err_t i2c_manager_add_device(i2c_manager_device_t *device);
esp_err_t i2c_manager_remove_device(i2c_manager_device_t *device);

esp_err_t i2c_manager_write(i2c_manager_device_t *device, const uint8_t *data, size_t data_len, uint32_t timeout_ms);
esp_err_t i2c_manager_read(i2c_manager_device_t *device, uint8_t *data, size_t data_len, uint32_t timeout_ms);
esp_err_t i2c_manager_write_read(i2c_manager_device_t *device,
                                 const uint8_t *write_buf,
                                 size_t write_len,
                                 uint8_t *read_buf,
                                 size_t read_len,
                                 uint32_t timeout_ms);

esp_err_t i2c_manager_write_reg(i2c_manager_device_t *device,
                                uint8_t reg,
                                const uint8_t *data,
                                size_t data_len,
                                uint32_t timeout_ms);
esp_err_t i2c_manager_read_reg(i2c_manager_device_t *device,
                               uint8_t reg,
                               uint8_t *data,
                               size_t data_len,
                               uint32_t timeout_ms);
esp_err_t i2c_manager_write_reg_u8(i2c_manager_device_t *device, uint8_t reg, uint8_t value, uint32_t timeout_ms);
esp_err_t i2c_manager_read_reg_u8(i2c_manager_device_t *device, uint8_t reg, uint8_t *value, uint32_t timeout_ms);

#endif
