/*
 * File: ressources.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-13
 * ----------------------------
 * Globale Queue-Handles zur Interprozesskommunikation mittel Intercom.
 * Handles müssen von den Modulen initialisiert werden.
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#pragma once

// Sensorik
QueueHandle_t xSensors;
#define xSensors_PRIORITY (7U)

// Control
QueueHandle_t xControl;
#define xControl_PRIORITY (6U)

// Remote
QueueHandle_t xRemote;
#define xRemote_PRIORITY (4U)

// Info
TaskHandle_t xInfo_handle;
#define xInfo_PRIORITY (1U)
QueueHandle_t xInfo_input;