#include "i2c_manager.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define I2C_MANAGER_TIMEOUT_MS_DEFAULT 100U

static const char *TAG = "i2c_manager";

static i2c_master_bus_handle_t s_bus_handle;
static SemaphoreHandle_t s_bus_lock;
static bool s_initialized;
static uint32_t s_default_speed_hz = 400000U;

static esp_err_t i2c_manager_lock(uint32_t timeout_ms)
{
    if (s_bus_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint32_t effective_timeout_ms = (timeout_ms == 0U) ? I2C_MANAGER_TIMEOUT_MS_DEFAULT : timeout_ms;
    if (xSemaphoreTake(s_bus_lock, pdMS_TO_TICKS(effective_timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static void i2c_manager_unlock(void)
{
    if (s_bus_lock != NULL) {
        xSemaphoreGive(s_bus_lock);
    }
}

static bool i2c_manager_device_is_ready(const i2c_manager_device_t *device)
{
    return (device != NULL) && device->is_added && (device->handle != NULL);
}

esp_err_t i2c_manager_init(const i2c_manager_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_initialized) {
        return ESP_OK;
    }
    if (config->clk_speed_hz == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    s_bus_lock = xSemaphoreCreateMutex();
    if (s_bus_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = config->port,
        .sda_io_num = config->sda_io_num,
        .scl_io_num = config->scl_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = config->glitch_ignore_cnt,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = config->enable_internal_pullup,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus_handle);
    if (err != ESP_OK) {
        vSemaphoreDelete(s_bus_lock);
        s_bus_lock = NULL;
        return err;
    }

    s_default_speed_hz = config->clk_speed_hz;
    s_initialized = true;

    ESP_LOGI(TAG,
             "I2C initialized: port=%d SDA=%d SCL=%d speed=%" PRIu32 "Hz",
             config->port,
             config->sda_io_num,
             config->scl_io_num,
             config->clk_speed_hz);
    return ESP_OK;
}

esp_err_t i2c_manager_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    esp_err_t err = i2c_del_master_bus(s_bus_handle);
    if (err != ESP_OK) {
        return err;
    }

    s_bus_handle = NULL;
    s_initialized = false;

    if (s_bus_lock != NULL) {
        vSemaphoreDelete(s_bus_lock);
        s_bus_lock = NULL;
    }

    return ESP_OK;
}

bool i2c_manager_is_initialized(void)
{
    return s_initialized;
}

esp_err_t i2c_manager_probe(uint8_t address, uint32_t timeout_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((address < 0x08U) || (address > 0x77U)) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t effective_timeout_ms = (timeout_ms == 0U) ? I2C_MANAGER_TIMEOUT_MS_DEFAULT : timeout_ms;

    esp_err_t err = i2c_manager_lock(effective_timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_probe(s_bus_handle, address, effective_timeout_ms);
    i2c_manager_unlock();
    return err;
}

esp_err_t i2c_manager_scan(i2c_manager_scan_result_t *result, uint32_t timeout_ms_per_addr)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(result, 0, sizeof(*result));

    for (uint8_t addr = 0x08U; addr <= 0x77U; ++addr) {
        const esp_err_t err = i2c_manager_probe(addr, timeout_ms_per_addr);
        if (err == ESP_OK) {
            if (result->count < (sizeof(result->addresses) / sizeof(result->addresses[0]))) {
                result->addresses[result->count++] = addr;
            }
            continue;
        }

        if (err == ESP_ERR_NOT_FOUND) {
            continue;
        }

        ESP_LOGW(TAG, "I2C scan probe error at 0x%02X: %s", addr, esp_err_to_name(err));
    }

    if (result->count == 0U) {
        ESP_LOGW(TAG, "I2C scan: no devices detected");
        return ESP_ERR_NOT_FOUND;
    }

    char list_buf[256];
    size_t used = 0U;
    list_buf[0] = '\0';
    for (size_t i = 0; i < result->count; ++i) {
        const int n = snprintf(list_buf + used,
                               sizeof(list_buf) - used,
                               "%s0x%02X",
                               (i == 0U) ? "" : ", ",
                               result->addresses[i]);
        if ((n <= 0) || ((size_t)n >= (sizeof(list_buf) - used))) {
            break;
        }
        used += (size_t)n;
    }

    ESP_LOGI(TAG, "I2C scan detected %u device(s): %s", (unsigned)result->count, list_buf);
    return ESP_OK;
}

esp_err_t i2c_manager_add_device(i2c_manager_device_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((device->address < 0x08U) || (device->address > 0x77U)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (device->is_added) {
        return ESP_OK;
    }
    if ((device->scl_speed_hz == 0U) || (device->scl_speed_hz > s_default_speed_hz)) {
        device->scl_speed_hz = s_default_speed_hz;
    }

    const esp_err_t lock_err = i2c_manager_lock(I2C_MANAGER_TIMEOUT_MS_DEFAULT);
    if (lock_err != ESP_OK) {
        return lock_err;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device->address,
        .scl_speed_hz = device->scl_speed_hz,
        .scl_wait_us = 0,
    };

    esp_err_t err = i2c_master_bus_add_device(s_bus_handle, &device_config, &device->handle);
    if (err == ESP_OK) {
        device->is_added = true;
    }

    i2c_manager_unlock();
    return err;
}

esp_err_t i2c_manager_remove_device(i2c_manager_device_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!device->is_added || (device->handle == NULL)) {
        return ESP_OK;
    }

    const esp_err_t lock_err = i2c_manager_lock(I2C_MANAGER_TIMEOUT_MS_DEFAULT);
    if (lock_err != ESP_OK) {
        return lock_err;
    }

    esp_err_t err = i2c_master_bus_rm_device(device->handle);
    if (err == ESP_OK) {
        device->handle = NULL;
        device->is_added = false;
    }

    i2c_manager_unlock();
    return err;
}

esp_err_t i2c_manager_write(i2c_manager_device_t *device, const uint8_t *data, size_t data_len, uint32_t timeout_ms)
{
    if (!i2c_manager_device_is_ready(device) || (data == NULL) || (data_len == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2c_manager_lock(timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_transmit(device->handle, data, data_len, timeout_ms);
    i2c_manager_unlock();
    return err;
}

esp_err_t i2c_manager_read(i2c_manager_device_t *device, uint8_t *data, size_t data_len, uint32_t timeout_ms)
{
    if (!i2c_manager_device_is_ready(device) || (data == NULL) || (data_len == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2c_manager_lock(timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_receive(device->handle, data, data_len, timeout_ms);
    i2c_manager_unlock();
    return err;
}

esp_err_t i2c_manager_write_read(i2c_manager_device_t *device,
                                 const uint8_t *write_buf,
                                 size_t write_len,
                                 uint8_t *read_buf,
                                 size_t read_len,
                                 uint32_t timeout_ms)
{
    if (!i2c_manager_device_is_ready(device) || (write_buf == NULL) || (write_len == 0U) || (read_buf == NULL) ||
        (read_len == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2c_manager_lock(timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_transmit_receive(device->handle, write_buf, write_len, read_buf, read_len, timeout_ms);
    i2c_manager_unlock();
    return err;
}

esp_err_t i2c_manager_write_reg(i2c_manager_device_t *device,
                                uint8_t reg,
                                const uint8_t *data,
                                size_t data_len,
                                uint32_t timeout_ms)
{
    if (!i2c_manager_device_is_ready(device)) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((data_len > 0U) && (data == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t local_buf[16];
    const size_t frame_len = data_len + 1U;
    uint8_t *frame = local_buf;

    if (frame_len > sizeof(local_buf)) {
        frame = (uint8_t *)malloc(frame_len);
        if (frame == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    frame[0] = reg;
    if (data_len > 0U) {
        memcpy(&frame[1], data, data_len);
    }

    esp_err_t err = i2c_manager_write(device, frame, frame_len, timeout_ms);
    if (frame != local_buf) {
        free(frame);
    }
    return err;
}

esp_err_t i2c_manager_read_reg(i2c_manager_device_t *device,
                               uint8_t reg,
                               uint8_t *data,
                               size_t data_len,
                               uint32_t timeout_ms)
{
    if ((data == NULL) || (data_len == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_manager_write_read(device, &reg, 1U, data, data_len, timeout_ms);
}

esp_err_t i2c_manager_write_reg_u8(i2c_manager_device_t *device, uint8_t reg, uint8_t value, uint32_t timeout_ms)
{
    return i2c_manager_write_reg(device, reg, &value, 1U, timeout_ms);
}

esp_err_t i2c_manager_read_reg_u8(i2c_manager_device_t *device, uint8_t reg, uint8_t *value, uint32_t timeout_ms)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_manager_read_reg(device, reg, value, 1U, timeout_ms);
}
