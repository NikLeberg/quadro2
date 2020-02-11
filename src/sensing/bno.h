/*
 * File: bno.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-02-09
 * ----------------------------
 * Public API für BNO080 Treiber.
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "esp_system.h"
#include "driver/gpio.h"


/** Einstellungen **/

#define BNO_DATA_RATE_IMU_US        1000000   // 20 ms
#define BNO_DATA_RATE_PRESSURE_US   1000000  // 100 ms
#define BNO_STARTUP_WAIT_MS         1000    // 1 s


/*
 * Function: bno_init
 * ----------------------------
 * Initialisiert Sensor, installiert Hintergrundtask und blockiert währenddem.
 *
 * uint8_t bnoAddr: BNO I2C Adresse
 * gpio_num_t bnoInterrupt: BNO Interrupt Pin (Data ready)
 * gpio_num_t bnoReset: BNO Reset Pin
 *
 * returns: false -> Erfolg, true -> Error
 */
bool bno_init(uint8_t address, gpio_num_t interruptPin, gpio_num_t resetPin);
