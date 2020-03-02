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
#define GPS_SET_HOME_MIN_DOP    10.0f // 10 m


/*
 * Function: gps_init
 * ----------------------------
 * Initialisiert Sensor und startet zyklisches Update.
 * 
 * gpio_num_t rxPin: UART Data-In
 * gpio_num_t txPin: UART Data-Out
 *
 * returns: false -> Erfolg, true -> Error
 */
bool gps_init(gpio_num_t rxPin, gpio_num_t txPin);

/*
 * Function: gps_setHome
 * ----------------------------
 * Setze aktuelle Position als Homepunkt.
 * Wird erst beim nächsten guten Fix angewendet!
 */
void gps_setHome();