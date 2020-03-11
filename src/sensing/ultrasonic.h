/*
 * File: ultrasonic.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-04
 * ----------------------------
 * Interruptgesteuerter Ultaschallsensor
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "esp_system.h"
#include "driver/gpio.h"


/** Einstellungen **/

#define ULT_DATA_RATE_MS    100  // 100 ms
#define ULT_MAX_CM          5.0f // 5 m
#define ULT_NO_GROUND_COUNT 256
#define ULT_NO_GROUND_WAIT  1000 // 1 s


/*
 * Function: ult_init
 * ----------------------------
 * Initialisiert Sensor und startet zyklisches Update.
 *
 * gpio_num_t triggerPin: Trigger Pin, Output
 * gpio_num_t echoPin: Echo Pin, Input
 *
 * returns: false -> Erfolg, true -> Error
 */
bool ult_init(gpio_num_t triggerPin, gpio_num_t echoPin);
