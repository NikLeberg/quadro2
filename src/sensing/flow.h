/*
 * File: flow.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-07-20
 * ----------------------------
 * Mateksys Optical Flow Sensor basierend auf PMW3901.
 * Kommuniziert über Multiwii Serial Protokoll Version 2 (MSPv2).
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "esp_system.h"
#include "driver/gpio.h"
#include "sensor_types.h"


/** Einstellungen **/

#define FLOW_UART   UART_NUM_2


/*
 * Function: flow_init
 * ----------------------------
 * Initialisiert Sensor, installiert Hintergrundtask und blockiert währenddem.
 *
 * gpio_num_t uartRxPin: RX Pin der UART-Verbindung
 *
 * returns: false -> Erfolg, true -> Error
 */
bool flow_init(gpio_num_t uartRxPin);
