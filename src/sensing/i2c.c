/*
 * File: i2c.c
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2019-12-06
 * ----------------------------
 * I2C-Busfunktionen als Wrapper für ESP-IDF i2c-Treiber.
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


/** Interne Abhängigkeiten **/

#include "i2c.h"
#include "resources.h"


/** Variablendeklaration **/

static SemaphoreHandle_t sI2C;


/** Implementierung **/

bool i2c_init(gpio_num_t scl, gpio_num_t sda) {
    i2c_config_t i2c_config;
    i2c_config.mode           = I2C_MODE_MASTER;
    i2c_config.sda_io_num     = sda;
    i2c_config.sda_pullup_en  = GPIO_PULLUP_ENABLE;
    i2c_config.scl_io_num     = scl;
    i2c_config.scl_pullup_en  = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = I2C_CLOCK;
    // I2C Installieren
    if (i2c_param_config(I2C_NUM, &i2c_config)) return true;
    if (i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0)) return true;
    if (i2c_set_timeout(I2C_NUM, (1ULL << 20) - 1)) return true; // ca 13 ms Timeout
    // SCL Dutycycle des Drivers korrigieren (idf setzt 50 %, gemäss Spezifikation aber maximal 33 % zulässig)
    uint32_t cycleThird = APB_CLK_FREQ / I2C_CLOCK / 3;
    if (i2c_set_period(I2C_NUM, cycleThird, cycleThird * 2)
    || i2c_set_start_timing(I2C_NUM, cycleThird, cycleThird)
    || i2c_set_stop_timing(I2C_NUM, cycleThird, cycleThird)
    || i2c_set_data_timing(I2C_NUM, cycleThird, cycleThird)) return true;
    // Sempahor einrichten
    sI2C = xSemaphoreCreateMutex();
    if (!sI2C) return true;
    return false;
}

bool i2c_write(uint8_t deviceAddr, uint8_t* pData, size_t dataLength) {
    i2c_cmd_handle_t cmd;
    esp_err_t err;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, deviceAddr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, pData, dataLength, true);
    i2c_master_stop(cmd);
    if (xSemaphoreTake(sI2C, I2C_LOCK_TIMEOUT_MS / portTICK_RATE_MS) == pdFALSE) return true;
    err = i2c_master_cmd_begin(I2C_NUM, cmd, I2C_BUS_TIMEOUT_MS / portTICK_RATE_MS);
    xSemaphoreGive(sI2C);
    i2c_cmd_link_delete(cmd);
    return err;
}

bool i2c_read(uint8_t deviceAddr, uint8_t* pData, size_t dataLength) {
    i2c_cmd_handle_t cmd;
    esp_err_t err;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, deviceAddr << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, pData, dataLength, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    if (xSemaphoreTake(sI2C, I2C_LOCK_TIMEOUT_MS / portTICK_RATE_MS) == pdFALSE) return true;
    err = i2c_master_cmd_begin(I2C_NUM, cmd, I2C_BUS_TIMEOUT_MS / portTICK_RATE_MS);
    xSemaphoreGive(sI2C);
    i2c_cmd_link_delete(cmd);
    return err;
}