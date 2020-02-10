/*
 * File: i2c.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2019-12-06
 * ----------------------------
 * I2C-Busfunktionen als Wrapper für ESP-IDF i2c-Treiber.
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"


/** Einstellungen **/

#define I2C_NUM         I2C_NUM_0
#define I2C_CLOCK       440000


/*
 * Function: i2c_init
 * ----------------------------
 * Aktiviert I2C-Master auf gegebenen Pins.
 *
 * gpio_num_t scl: Serial Clock
 * gpio_num_t sda: Serial Data
 *
 * returns: false -> Erfolg, true -> Error
 */
bool i2c_init(gpio_num_t scl, gpio_num_t sda);

/*
 * Function: i2c_write
 * ----------------------------
 * Schreibe Bytes in Slave.
 *
 * uint8_t deviceAddr: Adresse des Slaves
 * uint8_t* pData: Pointer zu Daten
 * size_t dataLength: Anzahl zu sendender Bytes
 *
 * returns: false -> Erfolg, true -> Error
 */
bool i2c_write(uint8_t deviceAddr, uint8_t* pData, size_t dataLength);

/*
 * Function: i2c_read
 * ----------------------------
 * Lese Bytes aus Slave.
 *
 * uint8_t deviceAddr: Adresse des Slaves
 * uint8_t* pData: Pointer zu Daten
 * size_t dataLength: Anzahl der zu empfangender Bytes
 *
 * returns: false -> Erfolg, true -> Error
 */
bool i2c_read(uint8_t deviceAddr, uint8_t* pData, size_t dataLength);