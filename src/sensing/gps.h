/*
 * File: gps.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-15
 * ----------------------------
 * GPS u-blox Parser für Beitian BN-880Q Sensor
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "esp_system.h"
#include "driver/gpio.h"


/** Einstellungen **/

#define GPS_UART                UART_NUM_1
#define GPS_DATA_RATE_MS        1000 // 1 Hz


/*
 * Function: gps_init
 * ----------------------------
 * Initialisiert Sensor und startet zyklisches Update.
 * 
 * gpio_num_t rxPin: UART Data-In
 * gpio_num_t txPin: UART Data-Out
 * uint32_t *rate: Datenrate, reagiert bei Änderung automatisch
 *
 * returns: false -> Erfolg, true -> Error
 */
bool gps_init(gpio_num_t rxPin, gpio_num_t txPin, uint32_t rate);

/*
 * Function: gps_updateRate
 * ----------------------------
 * Setze Sensorreport auf gewünschte Datenraten.
 *
 * uint32_t rate: neue Datenrate
 */
void gps_updateRate(uint32_t rate);
