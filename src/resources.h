/*
 * File: ressources.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-13
 * ----------------------------
 * Globale Ressourcen- und Task-Handles zur Interprozesskommunikation.
 * Handles müssen von den Modulen initialisiert werden.
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#pragma once

// Physikalische Ressourcen
SemaphoreHandle_t sI2C;

// Virtuelle Ressourcen
/* keine */

/** Sensorik **/
// Main
TaskHandle_t xSensors_handle;
#define xSensors_PRIORITY (7U)
QueueHandle_t xSensors_input;
// Ultrasonic
TaskHandle_t xUlt_handle;
QueueHandle_t xUlt_input;
// GPS
TaskHandle_t xGps_handle;
QueueHandle_t xGps_input;
// BNO
TaskHandle_t xBno_handle;
QueueHandle_t xBno_input;

// Remote
TaskHandle_t xRemote_handle;
#define xRemote_PRIORITY (5U)
QueueHandle_t xRemote_input;

// Info
TaskHandle_t xInfo_handle;
#define xInfo_PRIORITY (1U)
QueueHandle_t xInfo_input;
