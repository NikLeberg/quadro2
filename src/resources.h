/*
 * File: ressources.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-13
 * ----------------------------
 *   Globale Ressourcen- und Task-Handles zur Interprozesskommunikation.
 *   Handles müssen von den Modulen initialisiert werden.
 */


/** Externe Abhängigkeiten **/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#pragma once

// Physikalische Ressourcen
SemaphoreHandle_t sI2C = NULL;

// Virtuelle Ressourcen
/* keine */

/** Sensorik **/
// Main
TaskHandle_t xSensors_handle = NULL;
#define xSensors_PRIORITY (7U)
QueueHandle_t xSensors_input = NULL;
// Ultrasonic
TaskHandle_t xUlt_handle = NULL;
QueueHandle_t xUlt_input = NULL;
// GPS
TaskHandle_t xGps_handle = NULL;
QueueHandle_t xGps_input = NULL;
// BNO
TaskHandle_t xBno_handle = NULL;
QueueHandle_t xBno_input = NULL;

// Remote
TaskHandle_t xRemote_handle = NULL;
#define xRemote_PRIORITY (6U)
QueueHandle_t xRemote_input = NULL;
