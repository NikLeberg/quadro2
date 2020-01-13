
#include "driver/i2c.h"

/*
 * File: i2c.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2019-12-06
 * ----------------------------
 *   I2C-Busfunktionen
 */


/** Externe Abhängigkeiten **/
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include "driver/i2c.h"

/** Interne Abhängigkeiten **/
#include "resourcesAndTasks.h"

#pragma once

static bool i2c_init(gpio_num_t scl, gpio_num_t sda) {
    i2c_config_t i2c_config;
    i2c_config.mode           = I2C_MODE_MASTER;
    i2c_config.sda_io_num     = sda;
    i2c_config.sda_pullup_en  = GPIO_PULLUP_ENABLE;
    i2c_config.scl_io_num     = scl;
    i2c_config.scl_pullup_en  = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = 440000;
    // I2C Installieren
    if (i2c_param_config(I2C_NUM_0, &i2c_config)) return true;
    if (i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0)) return true;
    if (i2c_set_timeout(I2C_NUM_0, (1ULL << 20) - 1)) return true; // ca 13 ms Timeout
    // Sempahor einrichten
    sI2C = xSemaphoreCreateMutex();
    if (!sI2C) return true;
    return false;
}

inline bool i2c_write(uint8_t deviceAddr, uint8_t* pData, size_t dataLength) {
    i2c_cmd_handle_t cmd;
    esp_err_t err;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, deviceAddr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, pData, dataLength, true);
    i2c_master_stop(cmd);
    if (xSemaphoreTake(sI2C, 100 / portTICK_RATE_MS) == pdFALSE) return true;
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
    xSemaphoreGive(sI2C);
    i2c_cmd_link_delete(cmd);
    return err;
}

inline bool i2c_read(uint8_t deviceAddr, uint8_t* pData, size_t dataLength) {
    i2c_cmd_handle_t cmd;
    esp_err_t err;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, deviceAddr << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, pData, dataLength, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    if (xSemaphoreTake(sI2C, 100 / portTICK_RATE_MS) == pdFALSE) return true;
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
    xSemaphoreGive(sI2C);
    i2c_cmd_link_delete(cmd);
    return err;
}